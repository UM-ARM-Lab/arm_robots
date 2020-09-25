from arm_robots.victor import Victor


def main():
    victor = Victor()

    group_name = 'both_arms'
    end_effector = 'left_tool_placeholder'
    end_position = [0.7, 0.0, 1.0]
    victor.follow_jacobian_to_position(group_name, end_effector, end_position)


if __name__ == '__main__':
    main()
