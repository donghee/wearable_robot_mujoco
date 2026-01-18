upperlimb = None

def setup():
    upperlimb.init(rep_count=20, freq=60)

def loop():
    # task 1
    #  if upperlimb.get_target_angle() > 90:
    #      upperlimb.set_velocity(-1.2)
    #  else:
    #      upperlimb.set_velocity(1.2)

    # task 2
    delta_velocity = upperlimb.get_velocity() - upperlimb.get_previous_velocity()
    acceleration = delta_velocity / upperlimb.DELTA_TIME
    acceleration = max(min(acceleration, 500.0), -500.0)
    delta_force = upperlimb.get_force() * 1000 / 9.8   # dimension change
    m = upperlimb.get_moment() # moment of inertia 관성 계수
    k = upperlimb.get_spring() # spring 계수
    c = upperlimb.get_damping() # damping 계수

    if upperlimb.get_target_angle() > 90:
        velocity = -0.6 + (delta_force - m * acceleration - k * (upperlimb.get_position() - upperlimb.get_target_angle())) / c # flexion
    else:
        velocity = 5.2  # extension
    upperlimb.set_velocity(velocity)
