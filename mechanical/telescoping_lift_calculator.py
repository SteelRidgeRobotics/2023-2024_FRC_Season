# Telescoping Lift Calculator

# The telescoping lift consists of a series of consecutively smaller square tubes, one
# stacked inside the next with the largest tube at the bottom of the stack. A bearing
# block at the bottom of each inner tube support the tube against the inside of the
# next larger tube.
#
#
#                     |<------------------- Yn ------------------->|
#         |<--- A --->|<-- B -->|                                  |<----- C ----->|
# ---------------------         |                                  |               |
#         =======================-----------------------------------               |
#                                ================-----------------------------------
#                                ================-----------------------------------
#         =======================-----------------------------------               |
# ---------------------                                            |               |
#         |<---------------------- EXTENDED ---------------------->|               |
#                                |<------------------ RETRACTED ------------------>|

# The portion of tube (----) and bearing block (====) left in the previous
# stage when the lift is fully extended
A = 0.875

# The top of the bearing block relative to the top of next outer tube
B = 0.75

# The height the tube extends beyond the next outer tube when the lift is retracted
C = 2.000


def tube_length_n(y1, n):
    """Returns the length of n-th tube in the lift given the length of the lowest tube"""
    if n == 1:
        return y1
    elif n >= 2:
        return y1 + (n-1) * C - (n-1) * A - (n-2) * B
    else:
        ValueError("n must be a positive integer")


def y1_tube_length(max_height, n_stages):
    """Returns the length of the lowest tube in a n stage lift of a given maximum height"""
    term = sum(range(n_stages+1)) * (C - A) - sum(range(n_stages)) * B
    return (max_height - term) / (n_stages + 1)


def minimum_height(y1: float, n_stages: int):
    """Returns the minimum height of n stage lift with the specified lowest tube length"""
    return y1 + 2 * n_stages


def maximum_height(tubes: list[float]):
    """Returns the maximum height of a lift given a list of tubes lengths"""
    return sum(tubes)


def lift_report(max_height, n_stages):
    """Displays the parameters of an n-stage lift with the specified maximum height"""
    y1 = y1_tube_length(max_height, n_stages)
    tubes = [y1]
    for i in range(2, n_stages + 2):
        tubes.append(tube_length_n(y1, i))
    max_height = maximum_height(tubes)
    min_height = minimum_height(y1, n_stages)
    dynamic_range = max_height - min_height

    print(f"{n_stages} Stage Lift")
    print("----------------")
    for i, val in enumerate(tubes):
        print(f"Tube {i+1} exposed tube length is {val} inches")
    print(f"Total maximum height is {max_height} inches")
    print(f"Total minimum height is {min_height} inches")
    print(f"Total dynamic range is {dynamic_range} inches")
    print()


# Generate some reports
# lift_report(45.5, 3)
lift_report(46.5, 3)
# lift_report(45.5, 4)
# lift_report(46.25, 3)
# lift_report(47.0, 3)
# lift_report(47.5, 3)

# This is the paper model
# lift_report(27, 3)
# lift_report(24, 3)
