from math import sqrt, pi, sin, cos, atan2
import numpy as np
import matplotlib.pyplot as plt


def get_time(distance, acceleration):
    return sqrt(2 * pi * distance / acceleration)


def get_motion_path(t_x, t_y, a_x, a_y, H, h, b):
    a = []
    v = []
    s = []
    for t in np.arange(0, t_x + t_y + 0.00005, 0.00005):
        if t < t_y / 2:
            ay, vy, sy_ = get_path(a_y, t_y, t)
            sy = sy_ - H - h
            ax, vx, sx = 0, 0, -b / 2
        elif t < t_y:
            ay, vy, sy_ = get_path(a_y, t_y, t)
            sy = sy_ - H - h
            ax, vx, sx_ = get_path(a_x, t_x, t - t_y / 2)
            sx = sx_ - b / 2
        elif t < t_x:
            ay, vy, sy = 0, 0, -H
            ax, vx, sx_ = get_path(a_x, t_x, t - t_y / 2)
            sx = sx_ - b / 2
        elif t < t_x + t_y / 2:
            ay, vy, sy_ = get_path(-a_y, t_y, t - t_x)
            sy = sy_ - H
            ax, vx, sx_ = get_path(a_x, t_x, t - t_y / 2)
            sx = sx_ - b / 2
        else:
            ay, vy, sy_ = get_path(-a_y, t_y, t - t_x)
            sy = sy_ - H
            ax, vx, sx = 0, 0, b / 2
        a.append([ax, ay])
        v.append([vx, vy])
        s.append([sx, sy])
    return a, v, s


def get_path(a_max, T, t):
    a = a_max * sin(2 * pi / T * t)
    v = a_max * T / (2 * pi) * (1 - cos(2 * pi / T * t))
    s = a_max * T ** 2 / (2 * pi) * (t / T - 1 / (2 * pi) * sin(2 * pi / T * t))
    return a, v, s


def get_theta(l1, l2, x, y, e, i):
    sgn = [1, -1]
    A = -2 * l1 * y
    B = -2 * l1 * (x - sgn[i - 1] * e)
    C = x ** 2 + y ** 2 + l1 ** 2 + e ** 2 - l2 ** 2 - 2 * sgn[i - 1] * e * x
    theta = -atan2(B, A) + atan2(-C, sgn[i - 1] * sqrt(A ** 2 + B ** 2 - C ** 2))
    return theta


def get_theta_derivatives(a, v, s, l1, l2, e):
    a = np.array(a).T
    v = np.array(v).T
    r = np.array(s).T
    x = s[0]
    y = s[1]
    sgn = [1, -1]
    Q = np.array([[0, -1], [1, 0]])
    theta_1 = get_theta(l1, l2, x, y, e, 1)
    theta_2 = get_theta(l1, l2, x, y, e, 2)
    u_1 = np.array([cos(theta_1), sin(theta_1)]).T
    u_2 = np.array([cos(theta_2), sin(theta_2)]).T
    x_ = np.array([1, 0]).T
    w_1 = (r - sgn[0] * e * x_ - l1 * u_1) / l2
    w_2 = (r - sgn[1] * e * x_ - l1 * u_2) / l2
    J = (1 / l1) * np.array([w_1 / (w_1.T.dot(Q).dot(u_1)), w_2 / (w_2.T.dot(Q).dot(u_2))])
    H1 = (np.outer(w_1.T.dot(u_1) * w_1, w_1.T) + np.outer((l1 / l2) * u_1, u_1.T)) / (
            l1 ** 2 * ((w_1.T.dot(Q).dot(u_1)) ** 3))
    H2 = (np.outer(w_2.T.dot(u_2) * w_2, w_2.T) + np.outer((l1 / l2) * u_2, u_2.T)) / (
            l1 ** 2 * ((w_2.T.dot(Q).dot(u_2)) ** 3))
    f_v = np.array([v.T.dot(H1).dot(v), v.T.dot(H2).dot(v)]).T
    return [theta_1, theta_2], J.dot(v), J.dot(a) + f_v, J, f_v


def get_torque(theta, J, f_v, I, a, m, mr):
    t_a = (m * np.linalg.inv(J.T) + I * J).dot(a)
    t_v = I * f_v
    t_g = m * 9.81 * np.linalg.inv(J.T).dot(np.array([0, 1])) + mr * 9.81 * np.array([cos(theta[0]), cos(theta[1])]).T
    return t_a + t_v + t_g


def get_power(theta_dot, t):
    p1 = t[0] * theta_dot[0]
    p2 = t[1] * theta_dot[1]
    return [p1, p2]


def draw_plot(pic, x, y, t, label, t_total, x_label, y_label):
    pic.plot(t, x)
    pic.plot(t, y)
    pic.legend(label)
    pic.set_xlim([0, t_total])
    pic.set_xlabel(x_label)
    pic.set_ylabel(y_label)
    pic.grid(linestyle='--')


def get_lim(v, a, t, p):
    data_dict = {"Angular velocity": v, "Angular acceleration": a, "Torque": t, "Power": p}
    lim_dict = {}
    for key in data_dict.keys():
        lim_dict[key] = [
            [min([d[i] for d in data_dict[key]]), max([d[i] for d in data_dict[key]])] for i in range(2)
        ]
    return lim_dict


def get_result(param_tuple, file_path, file_name):
    l_1, l_2, e, n, m, mr, I, H, b, h, a_x, a_y = param_tuple
    n = n[0] / n[1]
    t_x = get_time(b, a_x)
    t_y = get_time(h, a_y)
    t_total = t_x + t_y
    p_x = a_x * t_x ** 2 / (2 * pi) * (t_y / 2 / t_x - 1 / (2 * pi) * sin(2 * pi / t_x * t_y / 2))
    t_list = np.arange(0, t_x + t_y + 0.00005, 0.00005)
    a, v, s = get_motion_path(t_x, t_y, a_x, a_y, H, h, b)
    theta_list = []
    theta_dot_list = []
    theta_double_dot_list = []
    torque_list = []
    power_list = []
    t_rated = [0, 0]
    p_rated = [0, 0]
    for i in range(len(a)):
        theta, theta_dot, theta_double_dot, J, f_v = get_theta_derivatives(a[i], v[i], s[i], l_1, l_2, e)
        torque = get_torque(theta, J, f_v, I, a[i], m, mr)
        power = get_power(theta_dot, torque)
        theta_list.append(theta)
        theta_dot_list.append(theta_dot)
        theta_double_dot_list.append(theta_double_dot)
        torque_list.append(torque)
        power_list.append(power)
        for z in range(2):
            t_rated[z] = t_rated[z] + (torque[z] / n) ** 2
            p_rated[z] = p_rated[z] + power[z] ** 2
    for i in range(2):
        t_rated[i] = sqrt(t_rated[i] / len(a))
        p_rated[i] = sqrt(p_rated[i] / len(a))
    lim_data = get_lim(theta_dot_list, theta_double_dot_list, torque_list, power_list)
    col_label = ["Angular velocity", "Angular acceleration", "Torque", "Power"]
    row_label = ["Link1", "Link2"]
    data = []
    for i in range(4):
        l1 = f"min:{round(lim_data[col_label[i]][0][0], 2)}    max:{round(lim_data[col_label[i]][0][1], 2)}"
        l2 = f"min:{round(lim_data[col_label[i]][1][0], 2)}    max:{round(lim_data[col_label[i]][1][1], 2)}"
        data.append([l1, l2])
    data = np.array(data).T.tolist()
    result_pic = plt.figure(figsize=(24, 36), dpi=120)
    result_pic.subplots_adjust(wspace=0.4, hspace=0.3)
    text1 = result_pic.add_subplot(711)
    text1.axis('off')
    text1.text(-0.05, 1, "RESULT", fontsize=60, verticalalignment='top', horizontalalignment='left')
    text1.text(-0.05, 0.6, "1. Trajectory Planing", fontsize=40, verticalalignment='top', horizontalalignment='left')
    text1.text(-0.05, 0.32,
               "The following is the change of acceleration, velocity and position over time and the key data are given:",
               fontsize=24, verticalalignment='top', horizontalalignment='left')
    text1.text(-0.02, 0.16, f"The total time: {round(t_total, 3)}s", fontsize=24, verticalalignment='top',
               horizontalalignment='left')
    text1.text(-0.02, 0,
               f"The unknown coordinates are: p_3=({round(-b / 2 + p_x, 3)},{-H}), p_4=({round(b / 2 - p_x, 3)},{-H})",
               fontsize=24, verticalalignment='top', horizontalalignment='left')
    draw_plot(result_pic.add_subplot(7, 3, 4), [i[0] for i in a], [i[1] for i in a], t_list, ['x', 'y'], t_total,
              't(s)', 'Acceleration(m/s^2)')
    draw_plot(result_pic.add_subplot(7, 3, 5), [i[0] for i in v], [i[1] for i in v], t_list, ['x', 'y'], t_total,
              't(s)', 'Velocity(m/s)')
    draw_plot(result_pic.add_subplot(7, 3, 6), [i[0] for i in s], [i[1] for i in s], t_list, ['x', 'y'], t_total,
              't(s)', 'Position(m)')
    text2 = result_pic.add_subplot(713)
    text2.axis("off")
    text2.text(-0.05, 0.5, "2. Inverse Kinematic Analysis", fontsize=40, verticalalignment='top',
               horizontalalignment='left')
    text2.text(-0.05, 0.2,
               "Through the inverse kinematics analysis, the changes of the angle, angular velocity and angular acceleration of the actuated proximal links relative to time can be obtained: ",
               fontsize=24, verticalalignment='top', horizontalalignment='left', wrap=True)
    draw_plot(plt.subplot(7, 3, 10), [theta[0] for theta in theta_list], [theta[1] for theta in theta_list], t_list,
              ["theta1", "theta2"], t_total, 't(s)', 'Angle(rad)')
    draw_plot(plt.subplot(7, 3, 11), [theta[0] for theta in theta_dot_list], [theta[1] for theta in theta_dot_list],
              t_list, ["theta1", "theta2"], t_total, 't(s)', 'Angular velocity(rad/s)')
    draw_plot(plt.subplot(7, 3, 12), [theta[0] for theta in theta_double_dot_list],
              [theta[1] for theta in theta_double_dot_list], t_list, ["theta1", "theta2"], t_total, 't(s)',
              'Angular acceleration(rad/s^2)')
    text3 = result_pic.add_subplot(715)
    text3.axis("off")
    text3.text(-0.05, 0.4, "3. Inverse Dynamic Analysis", fontsize=40, verticalalignment='top',
               horizontalalignment='left')
    text3.text(-0.05, 0.1,
               "The variation of the actuated proximal links torque and power with time is obtained by inverse dynamics analysis",
               fontsize=24, verticalalignment='top', horizontalalignment='left', wrap=True)
    draw_plot(plt.subplot(7, 3, 16), [t[0] for t in torque_list], [t[1] for t in torque_list], t_list,
              ["theta1", "theta2"], t_total, 't(s)', 'Torque(Nm)')
    draw_plot(plt.subplot(7, 3, 17), [p[0] for p in power_list], [p[1] for p in power_list], t_list,
              ["theta1", "theta2"], t_total, 't(s)', 'Power(Nm/s)')
    table = result_pic.add_subplot(7, 1, 7)
    table.axis('off')
    table.text(-0.05, 0.7, "Through the above analysis, we can get some key data:", fontsize=24,
               verticalalignment='top', horizontalalignment='left')
    table.text(-0.02, 0.5,
               f"The rated torque of the servomotors: torque_motor1={round(t_rated[0], 2)}Nm, , torque_motor2={round(t_rated[1], 2)}Nm",
               fontsize=24, verticalalignment='top', horizontalalignment='left')
    table.text(-0.02, 0.3,
               f"The rated power of the servomotors: power_motor1={round(p_rated[0], 2)}Nm/s, , power_motor2={round(p_rated[1], 2)}Nm/s",
               fontsize=24, verticalalignment='top', horizontalalignment='left')
    data_table = table.table(cellText=data, rowLabels=row_label, colLabels=col_label, rowLoc='center', colLoc='center',
                             loc="bottom")
    data_table.set_fontsize(24)
    data_table.scale(1, 4)
    file_path = f"{file_path}/{file_name}.png"
    result_pic.savefig(file_path, dpi=300)


if __name__ == "__main__":
    param_tuple_ = (0.35, 0.85, 0.08, (22, 1), 1.8, 0.45, 0.22, 0.6, 0.7, 0.025, 140.0, 35.0)
    path = ['/Users/xiaoen/Documents/学习/python/2D', 'result']
    get_result(param_tuple_, path)
