from mpc_controller import mpc_controller
import time
import matplotlib.pyplot as plt
# Lead Vehicle
v_lead = 25
a_lead = 1.25
acc_range = 100


def sample_model(v, u, dt):
    return [v*dt + (u*(dt**2)/2), v + (u*dt)]


def lead_car_speeds():
    mpcControl = mpc_controller()
    rel_dist = 100
    v0 = 20.0
    a0 = 2.0
    v_lead = 25.0
    v_max = 30.0
    safe_stop_dist = 15.0
    dt = 0.5
    N = 100

    current = time.time()
    t = [time.time() - current]
    r = [rel_dist]
    u = [a0]
    v = [v0]
    outOfRange = False
    i = 0
    # Run MPC for 1 Minute
    while time.time()-current < 60:
        i = i + 1
        t.append(time.time() - current)
        dt_sim = t[i] - t[i-1]
        print(f"V_LEAD: {v_lead}")
        print(f"EGO_VEHCILE V {v0}")
        print(f"REL_DISTANCE {rel_dist}")
        U, J, target_vel = mpcControl.mpc(
            rel_dist, v0, a0, v_lead, v_max, safe_stop_dist, dt_sim, N)
        # The lead vehicle changes its speed based on a simple equations

        updatParams = sample_model(v0, U, dt_sim)
        v_lead = v_lead + (a_lead*dt_sim)
        if v_lead >= 50:
            v_lead = 50

        # rel_dist = (rel_dist - updatParams[0]) + \
        #     (v_lead*0.5 + (a_lead*(0.5 ** 2)/2))
        a0 = U
        v0 = updatParams[1]
        dl = (v_lead*dt_sim) + (0.5*a_lead*dt_sim*dt_sim) + rel_dist
        d0 = (v0*dt_sim) + (0.5*a0*dt_sim*dt_sim)
        rel_dist = dl - d0
        if rel_dist > acc_range:
            rel_dist = safe_stop_dist
            outOfRange = True
        if outOfRange:
            rel_dist = safe_stop_dist
        r.append(rel_dist)
        u.append(U)
        v.append(v0)
        print(f"ACCELERATION: {a0}")
        print(f"Target V {target_vel}")
        print("DONE")

    plt.figure(1)
    plt.plot(t, r)
    plt.xlabel('Time [s]')
    plt.ylabel('Relative Distance [m]')
    plt.title('Relative Distance between Ego and Lead Vehicle')

    plt.figure(2)
    plt.plot(t, u)
    plt.xlabel('Time [s]')
    plt.ylabel('Conrol Signal U')
    plt.title('Conrrol Signal U vs time')
    plt.show()

    plt.figure(3)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('velocity')
    plt.title('velocity vs time')
    plt.show()


def lead_car_stops():
    mpcControl = mpc_controller()
    rel_dist = 100
    v0 = 30.0
    a0 = 2.0
    v_lead = 25.0
    v_max = 30.0
    safe_stop_dist = 15.0
    dt = 0.5
    N = 100

    current = time.time()
    t = [time.time() - current]
    r = [rel_dist]
    u = [a0]
    v = [v0]
    outOfRange = False
    i = 0
    # Run MPC for 1 Minute
    while time.time()-current < 30:
        i = i + 1
        t.append(time.time() - current)
        dt_sim = t[i] - t[i-1]
        print(f"V_LEAD: {v_lead}")
        print(f"EGO_VEHCILE V {v0}")
        print(f"REL_DISTANCE {rel_dist}")
        U, J, target_vel = mpcControl.mpc(
            rel_dist, v0, a0, v_lead, v_max, safe_stop_dist, dt_sim, N)
        # The lead vehicle changes its speed based on a simple equations

        updatParams = sample_model(v0, U, dt_sim)
        v_lead = v_lead - (a_lead*dt_sim)
        if v_lead <= 0:
            v_lead = 0

        # rel_dist = (rel_dist - updatParams[0]) + \
        #     (v_lead*0.5 + (a_lead*(0.5 ** 2)/2))
        a0 = U
        v0 = updatParams[1]
        dl = (v_lead*dt_sim) + (0.5*a_lead*dt_sim*dt_sim) + rel_dist
        d0 = (v0*dt_sim) + (0.5*a0*dt_sim*dt_sim)
        rel_dist = dl - d0
        if rel_dist > 150:
            rel_dist = safe_stop_dist
            outOfRange = True
        if outOfRange:
            rel_dist = safe_stop_dist
        r.append(rel_dist)
        u.append(U)
        v.append(v0)
        print(f"ACCELERATION: {a0}")
        print(f"Target V {target_vel}")
        print("DONE")

    plt.figure(1)
    plt.plot(t, r)
    plt.xlabel('Time [s]')
    plt.ylabel('Relative Distance [m]')
    plt.title('Relative Distance between Ego and Lead Vehicle')

    plt.figure(2)
    plt.plot(t, u)
    plt.xlabel('Time [s]')
    plt.ylabel('Conrol Signal U')
    plt.title('Conrrol Signal U vs time')
    plt.show()

    plt.figure(3)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('velocity')
    plt.title('velocity vs time')
    plt.show()


# lead_car_speeds()
lead_car_stops()
