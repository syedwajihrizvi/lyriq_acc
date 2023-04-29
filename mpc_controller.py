import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


class mpc_controller:
    #Object definition for PID backup
    def __init__(self):
        self.lastError = 0
        self.integral = 0

#vehicle model used for the predition simulation
    def vehicle_model(x, u, v_lead, a_lead, soc, dt):
#map the soc to the reference data for OCV
        SOC_curve = np.loadtxt('SOC_curve.csv', delimiter=',')
        OCV_curve = np.loadtxt('OCV_curve.csv', delimiter=',')
        voltage = np.interp(soc, SOC_curve, OCV_curve)
#define vehicle constants
        DIFF = 0
        MASS = 2600
        AREA = 1.1
        CD = 0.38
        RHO = 1.225
        R = 0.733/2
        NUM_PARALLEL = 24
        MECH_POWER = 367749

#perform force analysis to find total requested power
        fx = MASS*u
        fdrag = (1/2)*CD*RHO*(x[1]**2)*AREA
        wheel_force = (fx + fdrag)/(2+(2*DIFF))
        front_torque = 2*(wheel_force)*R
        rear_torque = 2*(DIFF*wheel_force)*R
        wheel_speed = x[1]/R
        front_motor_power = front_torque*wheel_speed
        rear_motor_power = rear_torque*wheel_speed
        total_power = (front_motor_power+rear_motor_power)/NUM_PARALLEL
        throttle = ca.if_else(
            total_power > 0, (front_motor_power+rear_motor_power)/MECH_POWER, 0)
        i = ca.if_else(total_power > 0, total_power/voltage, 0)

        rel_dist_change = x[0]+((v_lead*dt) + ((1/2) *
                                               a_lead*dt**2) - ((x[1]*dt) + ((1/2)*u*dt**2)))
        v_change = x[1] + u*dt
        v_lead_change = v_lead + (a_lead*dt)

        return ca.horzcat(rel_dist_change, v_change), v_lead_change, i, throttle

#The inital dynamics function used, not used currently
    def f(x, u, v_lead):
        return ca.horzcat((v_lead - x), u)

#prediction simulation function
    def predict_trajectory(rel_dist, v0, a0, v_lead, a_lead, soc, dt, N, opti):
#define casadi opti varibales for problem
        X = opti.variable(N+1, 2)#state vectors X[0,:] - Position, X[1,:] - velocity
        LEAD = opti.variable(N+1, 1)  #Lead velocity
        U = opti.variable(N, 1)#Accleration control variable
        THROTTLE = opti.variable(N, 1)
        J = opti.variable(N-1, 1)#jerk
        BATT_POWER_USED = opti.variable(N, 1)

#define all of the equality and inequality constraints
        opti.subject_to(X[0, 0] == rel_dist)
        opti.subject_to(X[0, 1] == v0)
        opti.subject_to(LEAD[0, 0] == v_lead)

#loop through all value sin the vectors for variable definition
        for i in range(N):
            # opti.subject_to(U[i] >= -5)
            # opti.subject_to(U[i] <= 5)
            opti.subject_to(THROTTLE[i] <= 1) #throttle cannot exceed 100% press
            opti.subject_to(THROTTLE[i] >= 0)
            # opti.subject_to(X[i+1, 1] >= 0)

            #find the next state based off the prediction simulator
            next_state, LEAD_change, power_draw, throttle_command = mpc_controller.vehicle_model(
                X[i, :], U[i], LEAD[i], a_lead, soc, dt)
            #equality constraints
            opti.subject_to(X[i+1, :] == next_state)
            opti.subject_to(LEAD[i+1] == LEAD_change)
            opti.subject_to(BATT_POWER_USED[i] == power_draw)
            opti.subject_to(THROTTLE[i] == throttle_command)
            # opti.subject_to(X[i+1, :] == X[i, :] +
            #                 mpc_controller.f(X[i, 1], U[i], v_lead)*dt)

            #Define jerk as the derivative of acceleration and apply constraints
            if i < N-1:
                opti.subject_to(J[i] <= 4)
                opti.subject_to(J[i] >= -4)
                if i == 0:
                    opti.subject_to(J[i] == (U[i] - a0)/dt)
                else:
                    opti.subject_to(J[i] == (U[i] - U[i-1])/dt)
        return X, U, J, BATT_POWER_USED, THROTTLE

    def mpc(self, rel_dist, v0, a0, v_lead, a_lead, v_max, soc, dt, N):
        safe_stop_dist =max(2*v0, 5.0)
        ACC_RANGE = 100
        if v_lead > v_max:
            rel_dist = safe_stop_dist
            v_lead = v_max
        if rel_dist > ACC_RANGE:
            rel_dist = safe_stop_dist
            v_lead = v_max
        # Set up optimization problem
        opti = ca.Opti()
  
        # Set up decision variables and constraints
        X, U, J, discharge, throttle = mpc_controller.predict_trajectory(
            rel_dist, v0, a0, v_lead, a_lead, soc, dt, N, opti)

        #define cost function and penalty weights
        w1 = 1000.0
        w2 = 2000.0
        w3 = 20.0
        w4 = 50.0
        w5 = 5.0
        w6 = 0.1
        opti.minimize(w1*ca.sumsqr(ca.fabs(U)) +
                      + w2*ca.sumsqr(ca.fabs(J)) +
                      w3*ca.sumsqr(ca.fabs(X[:, 0] - safe_stop_dist)) + w4*ca.sumsqr(ca.if_else(X[:, 1] > v_max, 100*ca.fabs(v_max-X[:, 1]),ca.fabs(v_max-X[:, 1]))) +
                      w5*ca.sumsqr(ca.if_else(X[:, 0] < safe_stop_dist, safe_stop_dist-X[:, 0], 0))
                      + w6*ca.sumsqr(discharge))
        
        #calculate all of the parameters for the backup PID 
        error =  rel_dist - safe_stop_dist
        self.integral = self.integral + (error*dt)
        if self.lastError == 0 and self.integral == 0:     
            derivative = 0
        else:
            derivative = (error - self.lastError)/dt
        self.lastError = error

        #try catch block to solve the optimization problem. To view the prediction results uncomment the plotting lines
        try:
            opti.solver('ipopt')
            sol = opti.solve()
            # tgrid = np.linspace(0, (N+1)*dt, N+1)
            # plt.figure()
            # plt.plot(tgrid, sol.value(X)[:, 0], '-', label='position')
            # plt.xlabel('Time [s]')
            # plt.ylabel('Relative Distance [m]')
            # plt.title('Relative Distance between Ego and Lead Vehicle')

            # plt.figure()
            # plt.plot(tgrid, sol.value(X)[:, 1], '-', label='velocity')
            # plt.xlabel('Time [s]')
            # plt.ylabel('Velocity [m/s]')
            # plt.title('Velocity of the Ego Vehicle')

            # plt.figure()
            # u = sol.value(U)
            # tgrid = np.linspace(0, (N)*dt, N)
            # plt.plot(tgrid, u, '-', label='acceleration')
            # plt.xlabel('Time [s]')
            # plt.ylabel('Acceleration [m/s^2]')
            # plt.title('Acceleration of the Ego Vehicle')

            # plt.figure()
            # plt.plot(tgrid, sol.value(throttle), '-', label='throttle percent')
            # plt.xlabel('Time [s]')
            # plt.ylabel('percent pressed')
            # plt.title('throttle of the Ego Vehicle')

            # plt.figure()
            # plt.plot(tgrid, sol.value(discharge), '-', label='current')
            # plt.xlabel('Time [s]')
            # plt.ylabel('Discharge (A)')
            # plt.title('Discharge of the Ego Vehicle')

            # plt.figure()
            # tgrid = np.linspace(0, (N-1)*dt, N-1)
            # plt.plot(tgrid, sol.value(J), '-', label='Jerk')
            # plt.xlabel('Time [s]')
            # plt.ylabel('Jerk [m/s^2]')
            # plt.title('Jerk of the Ego Vehicle')

            # plt.legend()
            # plt.show()'

            #return U and J in the solution
            return sol.value(U)[0], sol.value(J)[0]
        except:
            #exception block to clauclate acceleration based of PID controller
            P = .1
            I = 0.000
            D = .7
            anew = P*error + I*self.integral + D*derivative
            anew = min(anew,5)
            return anew, (anew - a0)/dt