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
    soc = 1

    current = time.time()
    t = [time.time() - current]
    r = [rel_dist]
    u = [a0]
    v = [v0]
    vl = [v_lead]
    outOfRange = False
    i = 0
    # Run MPC for 1 Minute
    while time.time()-current < 15:
        i = i + 1
        t.append(time.time() - current)
        dt_sim = t[i] - t[i-1]
        print(f"V_LEAD: {v_lead}")
        print(f"EGO_VEHCILE V {v0}")
        print(f"REL_DISTANCE {rel_dist}")
        U, J = mpcControl.mpc(
            rel_dist, v0, a0, v_lead,a_lead, v_max, soc, safe_stop_dist, dt_sim, N)
        # The lead vehicle changes its speed based on a simple equations

        updatParams = sample_model(v0, U, dt_sim)
        v_lead = v_lead + (a_lead*dt_sim)
        if v_lead >= 50:
            v_lead = 50

        a0 = U
        v0 = updatParams[1]
        dl = (v_lead*dt_sim) + (0.5*a_lead*dt_sim*dt_sim) + rel_dist
        d0 = (v0*dt_sim) + (0.5*a0*dt_sim*dt_sim)
        rel_dist = dl - d0

        r.append(rel_dist)
        u.append(U)
        v.append(v0)
        vl.append(v_lead)
        print(f"ACCELERATION: {a0}")
        print("DONE")

    plt.figure(1)
    plt.plot(t, r)
    plt.xlabel('Time [s]')
    plt.ylabel('Relative Distance [m]')
    plt.title('Relative Distance between Ego and Lead Vehicle')

    plt.figure(2)
    plt.plot(t, u)
    plt.xlabel('Time [s]')
    plt.ylabel('Control Signal U')
    plt.title('Control Signal U vs time')

    plt.figure(3)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('Ego Velocity')
    plt.title('Velocity vs Time')

    plt.figure(4)
    plt.plot(t, vl)
    plt.xlabel('Time [s]')
    plt.ylabel('Lead Velocity')
    plt.title('Velocity vs Time')
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
    soc = 1

    current = time.time()
    t = [time.time() - current]
    r = [rel_dist]
    u = [a0]
    v = [v0]
    vl = [v_lead]
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
        U, J = mpcControl.mpc(
            rel_dist, v0, a0, v_lead,a_lead, v_max,soc, safe_stop_dist, dt_sim, N)
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

        r.append(rel_dist)
        u.append(U)
        v.append(v0)
        vl.append(v_lead)
        print(f"ACCELERATION: {a0}")
        print("DONE")

    plt.figure(1)
    plt.plot(t, r)
    plt.xlabel('Time [s]')
    plt.ylabel('Relative Distance [m]')
    plt.title('Relative Distance between Ego and Lead Vehicle')

    plt.figure(2)
    plt.plot(t, u)
    plt.xlabel('Time [s]')
    plt.ylabel('Control Signal U')
    plt.title('Control Signal U vs time')

    plt.figure(3)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('Ego Velocity')
    plt.title('Velocity vs time')

    plt.figure(4)
    plt.plot(t, vl)
    plt.xlabel('Time [s]')
    plt.ylabel('Lead Velocity')
    plt.title('Lead Velocity vs time')
    plt.show()


def approach_stopped_car():
    mpcControl = mpc_controller()
    rel_dist = 300
    v0 = 30.0
    a0 = 2.0
    v_lead = 0
    v_max = 30.0
    safe_stop_dist = 15.0
    dt = 0.5
    N = 100
    soc = 1

    current = time.time()
    t = [time.time() - current]
    r = [rel_dist]
    u = [a0]
    v = [v0]
    vl = [v_lead]
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
        U, J = mpcControl.mpc(
            rel_dist, v0, a0, v_lead,a_lead, v_max,soc, safe_stop_dist, dt_sim, N)
        # The lead vehicle changes its speed based on a simple equations

        updatParams = sample_model(v0, U, dt_sim)
        v_lead = v_lead - (a_lead*dt_sim)
        if v_lead <= 0:
            v_lead = 0

        a0 = U
        v0 = updatParams[1]
        dl = (v_lead*dt_sim) + (0.5*a_lead*dt_sim*dt_sim) + rel_dist
        d0 = (v0*dt_sim) + (0.5*a0*dt_sim*dt_sim)
        rel_dist = dl - d0

        r.append(rel_dist)
        u.append(U)
        v.append(v0)
        vl.append(v_lead)
        print(f"ACCELERATION: {a0}")
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
    plt.title('Control Signal U vs time')

    plt.figure(3)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('Ego Velocity')
    plt.title('Velocity vs time')

    plt.figure(4)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('Lead Velocity')
    plt.title('Velocity vs time')
    plt.show()


def cut_in():
    mpcControl = mpc_controller()
    rel_dist = 75
    v0 = 30.0
    a0 = 2.0
    v_lead = 25
    v_max = 30.0
    safe_stop_dist = 15.0
    dt = 0.5
    N = 100
    soc = 1

    current = time.time()
    t = [time.time() - current]
    r = [rel_dist]
    u = [a0]
    v = [v0]
    vl = [v_lead]
    outOfRange = False
    i = 0
    cutin = False
    # Run MPC for 1 Minute
    while time.time()-current < 30:
        i = i + 1
        t.append(time.time() - current)
        dt_sim = t[i] - t[i-1]
        print(f"V_LEAD: {v_lead}")
        print(f"EGO_VEHCILE V {v0}")
        print(f"REL_DISTANCE {rel_dist}")
        U, J = mpcControl.mpc(
            rel_dist, v0, a0, v_lead, a_lead, v_max, soc, safe_stop_dist, dt_sim, N)
        # The lead vehicle changes its speed based on a simple equations

        updatParams = sample_model(v0, U, dt_sim)
        v_lead = v_lead + (a_lead*dt_sim)
        if v_lead >= 28:
            v_lead = 28

        a0 = U
        v0 = updatParams[1]
        dl = (v_lead*dt_sim) + (0.5*a_lead*dt_sim*dt_sim) + rel_dist
        d0 = (v0*dt_sim) + (0.5*a0*dt_sim*dt_sim)
        rel_dist = dl - d0

        if ((time.time()-current) > 20) and not(cutin):
            rel_dist = 300
            v_lead = 25
            cutin = True

        r.append(rel_dist)
        u.append(U)
        v.append(v0)
        vl.append(v_lead)
        print(f"ACCELERATION: {a0}")
        print("DONE")

    plt.figure(1)
    plt.plot(t, r)
    plt.xlabel('Time [s]')
    plt.ylabel('Relative Distance [m]')
    plt.title('Relative Distance between Ego and Lead Vehicle')

    plt.figure(2)
    plt.plot(t, u)
    plt.xlabel('Time [s]')
    plt.ylabel('Control Signal U')
    plt.title('Control Signal U vs time')

    plt.figure(3)
    plt.plot(t, v)
    plt.xlabel('Time [s]')
    plt.ylabel('Ego Velocity')
    plt.title('Velocity vs time')

    plt.figure(4)
    plt.plot(t, vl)
    plt.xlabel('Time [s]')
    plt.ylabel('Lead Velocity')
    plt.title('Velocity vs time')
    plt.show()


# lead_car_speeds()
# lead_car_stops()
# approach_stopped_car()
cut_in()
