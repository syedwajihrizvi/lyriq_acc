import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


class mpc_controller:
    def vehicle_model(x, u, v_lead, a_lead, soc, dt):
        SOC_curve = np.loadtxt('SOC_curve.csv', delimiter=',')
        OCV_curve = np.loadtxt('OCV_curve.csv', delimiter=',')
        voltage = np.interp(soc, SOC_curve, OCV_curve)
        DIFF = 0
        MASS = 2600
        AREA = 1.1
        CD = 0.38
        RHO = 1.225
        R = 0.733/2
        NUM_PARALLEL = 24
        MECH_POWER = 367749

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

    def f(x, u, v_lead):
        return ca.horzcat((v_lead - x), u)

    def predict_trajectory(rel_dist, v0, a0, v_lead, a_lead, target_dist, v_max, soc, dt, N, opti):
        X = opti.variable(N+1, 2)
        LEAD = opti.variable(N+1, 1)
        U = opti.variable(N, 1)
        THROTTLE = opti.variable(N, 1)
        J = opti.variable(N-1, 1)
        BATT_POWER_USED = opti.variable(N, 1)

        opti.subject_to(X[0, 0] == rel_dist)
        opti.subject_to(X[0, 1] == v0)
        opti.subject_to(LEAD[0, 0] == v_lead)

        k_p = 1.0
        for i in range(N):
            # opti.subject_to(U[i] >= -5)
            # opti.subject_to(U[i] <= 5)
            opti.subject_to(THROTTLE[i] <= 1)
            opti.subject_to(THROTTLE[i] >= 0)
            opti.subject_to(X[i+1, 1] >= 0)
            next_state, LEAD_change, power_draw, throttle_command = mpc_controller.vehicle_model(
                X[i, :], U[i], LEAD[i], a_lead, soc, dt)
            opti.subject_to(X[i+1, :] == next_state)
            opti.subject_to(LEAD[i+1] == LEAD_change)
            opti.subject_to(BATT_POWER_USED[i] == power_draw)
            opti.subject_to(THROTTLE[i] == throttle_command)
            # opti.subject_to(X[i+1, :] == X[i, :] +
            #                 mpc_controller.f(X[i, 1], U[i], v_lead)*dt)

            if i < N-1:
                opti.subject_to(J[i] <= 4)
                opti.subject_to(J[i] >= -4)
                if i == 0:
                    opti.subject_to(J[i] == (U[i] - a0)/dt)
                else:
                    opti.subject_to(J[i] == (U[i] - U[i-1])/dt)
        return X, U, J, BATT_POWER_USED, THROTTLE

    def mpc(self, rel_dist, v0, a0, v_lead, a_lead, v_max, soc, safe_stop_dist, dt, N):
        ACC_RANGE = 100
        if v_lead > v_max:
            rel_dist = safe_stop_dist
            v_lead = v_max
        if rel_dist > ACC_RANGE:
            rel_dist = safe_stop_dist
            v_lead = v_max
        # Set up optimization problem
        opti = ca.Opti()

        # Set up time horizon and number of control intervals
        # T = opti.variable()
        # opti.subject_to(T >= 0.1)
        # opti.subject_to(T <= 10.0)
        # Set up decision variables and constraints
        X, U, J, discharge, throttle = mpc_controller.predict_trajectory(
            rel_dist, v0, a0, v_lead, a_lead, safe_stop_dist, v_max, soc, dt, N, opti)

        w1 = 30.0
        w2 = 40.0
        w3 = 20.0
        w4 = 10000.0
        w5 = 500.0
        w6 = 0.1

        opti.minimize(w1*ca.sumsqr(ca.fabs(U)) + w2*ca.sumsqr(ca.fabs(J)) +
                      w3*ca.sumsqr(ca.fabs(X[:, 0] - safe_stop_dist)) + w4*ca.sumsqr(ca.if_else(X[:, 1] > v_max, ca.fabs(v_max-X[:, 1]), 0)) +
                      w5*ca.sumsqr(ca.if_else(X[:, 0] < safe_stop_dist, safe_stop_dist-X[:, 0], 0))
                      + w6*ca.sumsqr(discharge))

        # opti.subject_to(X[:, 1] <= v_max)
        # opti.subject_to(X[:, 1] >= 0)

        # Set up initial conditions and solve the problem
        # opti.set_initial(T, 1.0)

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
            # plt.show()

            return sol.value(U)[0], sol.value(J)[0]
        except:
            return 0, (0-a0)/dt
