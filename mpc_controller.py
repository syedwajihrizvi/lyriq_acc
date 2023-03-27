import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


class mpc_controller:
    def f(x, u, v_lead):
        return ca.horzcat((v_lead - x), u)

    def predict_trajectory(rel_dist, v0, a0, v_lead, target_dist, v_max, dt, N, opti):
        X = opti.variable(N+1, 2)
        U = opti.variable(N, 1)
        J = opti.variable(N-1, 1)
        cost = 0
        target_vel = opti.variable(N+1, 1)
        opti.subject_to(X[0, 0] == rel_dist)
        opti.subject_to(X[0, 1] == v0)
        opti.subject_to(target_vel[0] == v0)

        k_p = 1.0
        for i in range(N):
            opti.subject_to(target_vel[i+1] == ca.if_else(
                (v_lead+(k_p*(X[i, 0]-target_dist))) > v_max, v_max, v_lead+(k_p*(X[i, 0]-target_dist))))
            opti.subject_to(U[i] >= -3)
            opti.subject_to(U[i] <= 3)
            opti.subject_to(X[i+1, 1] >= 0)
            opti.subject_to(X[i+1, :] == X[i, :] +
                            mpc_controller.f(X[i, 1], U[i], v_lead)*dt)

            if i < N-1:
                opti.subject_to(J[i] <= 4)
                opti.subject_to(J[i] >= -4)
                if i == 0:
                    opti.subject_to(J[i] == (U[i] - a0)/dt)
                else:
                    opti.subject_to(J[i] == (U[i] - U[i-1])/dt)
        return X, U, J, target_vel

    def mpc(self, rel_dist, v0, a0, v_lead, v_max, safe_stop_dist, dt, N):

        if v_lead > v_max:
            rel_dist = safe_stop_dist
            v_lead = v_max
            print("DID CHANGE")
        # Set up optimization problem
        opti = ca.Opti()

        # Set up time horizon and number of control intervals
        # T = opti.variable()
        # opti.subject_to(T >= 0.1)
        # opti.subject_to(T <= 10.0)
        # Set up decision variables and constraints
        X, U, J, target_vel = mpc_controller.predict_trajectory(
            rel_dist, v0, a0, v_lead, safe_stop_dist, v_max, dt, N, opti)

        w1 = 40.0
        w2 = 40.0
        w3 = 20.0
        w4 = 20000.0
        w5 = 1000.0
        opti.minimize(w1*ca.sumsqr(ca.fabs(U)) + w2*ca.sumsqr(ca.fabs(J)) +
                      w3*ca.sumsqr(ca.fabs(X[:, 0] - safe_stop_dist)) + w4*ca.sumsqr(ca.if_else(X[:, 1] > v_max, ca.fabs(v_max-X[:, 1]), 0)) + w5*ca.sumsqr(ca.if_else(X[:, 0] < safe_stop_dist, safe_stop_dist-X[:, 0], 0)))

        # opti.subject_to(X[:, 1] <= v_max)
        # opti.subject_to(X[:, 1] >= 0)

        # Set up initial conditions and solve the problem
        # opti.set_initial(T, 1.0)
        opti.solver('ipopt')
        sol = opti.solve()
        return sol.value(U)[0], sol.value(J)[0], sol.value(target_vel)[0]
        # Plot results
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
        # tgrid = np.linspace(0, (N-1)*dt, N-1)
        # plt.plot(tgrid, sol.value(J), '-', label='Jerk')
        # plt.xlabel('Time [s]')
        # plt.ylabel('Jerk [m/s^2]')
        # plt.title('Jerk of the Ego Vehicle')

        # plt.legend()
        # plt.show()
