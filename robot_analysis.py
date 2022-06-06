from tkinter import Tk, ttk, StringVar, Button
import PIL.Image as Image
import PIL.ImageTk as ImageTk
from tkinter.filedialog import askdirectory
import os
from analysis import get_result
import forzen_dir


def where_tk(tk_window, wid_per, hei_per):
    screenwidth = tk_window.winfo_screenwidth()
    screenheight = tk_window.winfo_screenheight()
    window_width = wid_per * screenwidth
    window_height = hei_per * screenheight
    return f"{int(window_width)}x{int(window_height)}+{int((screenwidth - window_width) / 2)}+{int((screenheight - window_height) / 2)}"


def selectPath(path):
    path_ = askdirectory()
    if path_ == "":
        path.get()
    else:
        path_ = path_.replace("/", "\\")  # 实际在代码中执行的路径为“\“ 所以替换一下
        path.set(path_)


def openPath(path):
    dir_ = os.path.dirname(path.get()+"\\")
    os.system('start ' + dir_)


def get_number(parm_list):
    param_list = []
    for parm in parm_list:
        param_list.append(parm.get())
    for i in range(len(parm_list)):
        if i == 3:
            param_list[i] = (int(param_list[i].split(":")[0]), int(param_list[i].split(":")[1]))
        else:
            param_list[i] = float(param_list[i])
    return tuple(param_list)


def main():
    # Create a window
    window = Tk()
    window.title("Kinematic and Dynamic Analyses of a 2-D Planar Robot")
    window.geometry(where_tk(window, 0.6, 0.7))
    window.resizable(False, False)

    # Create parameter frame
    parameter_frame = ttk.LabelFrame(
        window,
        text="Parameters",
    )
    parameter_frame.place(relx=0.05, rely=0.03, relwidth=0.4, relheight=0.7)
    structure_frame = ttk.LabelFrame(parameter_frame, text="Structure")
    inertia_frame = ttk.LabelFrame(parameter_frame, text="Inertia")
    motion_frame = ttk.LabelFrame(parameter_frame, text="Motion")
    structure_frame.place(relx=0.05, rely=0.02, relwidth=0.9, relheight=0.26)
    inertia_frame.place(relx=0.05, rely=0.31, relwidth=0.9, relheight=0.26)
    motion_frame.place(relx=0.05, rely=0.60, relwidth=0.9, relheight=0.35)
    # Parameter input
    # Structure
    label_l_1 = ttk.Label(structure_frame, text="l_1(m):")
    l_1_d = StringVar(value="0.35")
    entry_l_1 = ttk.Entry(structure_frame, textvariable=l_1_d)
    label_l_2 = ttk.Label(structure_frame, text="l_2(m):")
    l_2_d = StringVar(value="0.85")
    entry_l_2 = ttk.Entry(structure_frame, textvariable=l_2_d)
    label_e = ttk.Label(structure_frame, text="e(m):")
    e_d = StringVar(value="0.08")
    entry_e = ttk.Entry(structure_frame, textvariable=e_d)
    label_n = ttk.Label(structure_frame, text="n:")
    n_d = StringVar(value="22:1")
    entry_n = ttk.Entry(structure_frame, textvariable=n_d)
    label_l_1.place(relx=0.05, rely=0.05, relwidth=0.2, relheight=0.5)
    entry_l_1.place(relx=0.25, rely=0.15, relwidth=0.2, relheight=0.3)
    label_l_2.place(relx=0.55, rely=0.05, relwidth=0.2, relheight=0.5)
    entry_l_2.place(relx=0.75, rely=0.15, relwidth=0.2, relheight=0.3)
    label_e.place(relx=0.05, rely=0.5, relwidth=0.2, relheight=0.5)
    entry_e.place(relx=0.25, rely=0.6, relwidth=0.2, relheight=0.3)
    label_n.place(relx=0.55, rely=0.5, relwidth=0.2, relheight=0.5)
    entry_n.place(relx=0.75, rely=0.6, relwidth=0.2, relheight=0.3)
    # Inertia
    label_m = ttk.Label(inertia_frame, text="m(kg):")
    m_d = StringVar(value="1.8")
    entry_m = ttk.Entry(inertia_frame, textvariable=m_d)
    label_m_a_r_a = ttk.Label(inertia_frame, text="m_ar_a\n(kg·m):")
    m_a_r_a_d = StringVar(value="0.45")
    entry_m_a_r_a = ttk.Entry(inertia_frame, textvariable=m_a_r_a_d)
    label_I_A = ttk.Label(inertia_frame, text="I_A\n(kgm^2):")
    I_A_d = StringVar(value="0.22")
    entry_I_A = ttk.Entry(inertia_frame, textvariable=I_A_d)
    label_m.place(relx=0.05, rely=0.05, relwidth=0.2, relheight=0.5)
    entry_m.place(relx=0.25, rely=0.15, relwidth=0.2, relheight=0.3)
    label_m_a_r_a.place(relx=0.55, rely=0.05, relwidth=0.2, relheight=0.5)
    entry_m_a_r_a.place(relx=0.75, rely=0.15, relwidth=0.2, relheight=0.3)
    label_I_A.place(relx=0.05, rely=0.5, relwidth=0.2, relheight=0.5)
    entry_I_A.place(relx=0.25, rely=0.6, relwidth=0.2, relheight=0.3)
    # Motion
    label_H = ttk.Label(motion_frame, text="H(m):")
    H_d = StringVar(value="0.6")
    entry_H = ttk.Entry(motion_frame, textvariable=H_d)
    label_b = ttk.Label(motion_frame, text="b(m):")
    b_d = StringVar(value="0.7")
    entry_b = ttk.Entry(motion_frame, textvariable=b_d)
    label_h = ttk.Label(motion_frame, text="h(m):")
    h_d = StringVar(value="0.025")
    entry_h = ttk.Entry(motion_frame, textvariable=h_d)
    label_a_max_x = ttk.Label(motion_frame, text="a_{max,x}\n(m/s^2):")
    a_max_x_d = StringVar(value="140.0")
    entry_a_max_x = ttk.Entry(motion_frame, textvariable=a_max_x_d)
    label_a_max_y = ttk.Label(motion_frame, text="a_{max,y}\n(m/s^2):")
    a_max_y_d = StringVar(value="35.0")
    entry_a_max_y = ttk.Entry(motion_frame, textvariable=a_max_y_d)
    label_H.place(relx=0.05, rely=0.05, relwidth=0.2, relheight=0.3)
    entry_H.place(relx=0.25, rely=0.1, relwidth=0.2, relheight=0.2)
    label_b.place(relx=0.55, rely=0.05, relwidth=0.2, relheight=0.3)
    entry_b.place(relx=0.75, rely=0.1, relwidth=0.2, relheight=0.2)
    label_h.place(relx=0.05, rely=0.38, relwidth=0.2, relheight=0.3)
    entry_h.place(relx=0.25, rely=0.43, relwidth=0.2, relheight=0.2)
    label_a_max_x.place(relx=0.55, rely=0.38, relwidth=0.22, relheight=0.3)
    entry_a_max_x.place(relx=0.75, rely=0.43, relwidth=0.2, relheight=0.2)
    label_a_max_y.place(relx=0.05, rely=0.70, relwidth=0.22, relheight=0.3)
    entry_a_max_y.place(relx=0.25, rely=0.75, relwidth=0.2, relheight=0.2)
    # package
    # param = ["entry_l_1", "entry_l_2", "entry_e", "entry_n", "entry_m", "entry_m_a_r_a", "entry_I_A", "entry_H", "entry_b", "entry_h", "entry_a_max_x", "entry_a_max_y"]
    param = [entry_l_1, entry_l_2, entry_e, entry_n, entry_m, entry_m_a_r_a, entry_I_A, entry_H, entry_b, entry_h, entry_a_max_x, entry_a_max_y]
    # Create image frame
    example_frame = ttk.LabelFrame(
        window,
        text="Schematic diagram",
    )
    example_frame.place(relx=0.5, rely=0.03, relwidth=0.45, relheight=0.7)
    example_frame.update()
    pil_image = Image.open(forzen_dir.app_path() + "/Schematic.png")
    image_zoom = pil_image.resize((round(0.98 * example_frame.winfo_width()), round(0.936 * example_frame.winfo_height())), Image.ANTIALIAS)
    schematic_diagram = ImageTk.PhotoImage(image_zoom)
    schematic_label = ttk.Label(example_frame, image=schematic_diagram, padding=0)
    schematic_label.place(relx=0, rely=0, relwidth=1, relheight=1)

    # Create file frame
    file_frame = ttk.LabelFrame(
        window,
        text="File",
    )
    file_frame.place(relx=0.05, rely=0.75, relwidth=0.9, relheight=0.2)
    path = StringVar()
    path.set(os.path.abspath(""))
    save_label = ttk.Label(file_frame, text="Storage Path:")
    path_entry = ttk.Entry(file_frame, textvariable=path, state="readonly")
    file_label = ttk.Label(file_frame, text="File Name:")
    name_d = StringVar(value="result")
    name_entry = ttk.Entry(file_frame, textvariable=name_d)
    save_label.place(relx=0.02, rely=0.05, relwidth=0.12, relheight=0.4)
    file_label.place(relx=0.02, rely=0.5, relwidth=0.12, relheight=0.4)
    path_entry.place(relx=0.14, rely=0.05, relwidth=0.68, relheight=0.4)
    name_entry.place(relx=0.14, rely=0.5, relwidth=0.68, relheight=0.4)
    # file save
    # file_path = path_entry.get()
    # file_name = name_entry.get()
    file_path = 0
    file_name = 0

    def fun():
        nonlocal file_path, file_name
        file_path = path_entry.get()
        file_name = name_entry.get()
        file_path = file_path.replace("\\", "/")

    select_button = Button(file_frame, text="Select Path", command=lambda: selectPath(path))
    calculate_button = Button(file_frame, text="Calculate", command=lambda: [fun(), get_result(get_number(param), file_path, file_name)])
    select_button.place(relx=0.83, rely=0.05, relwidth=0.15, relheight=0.4)
    calculate_button.place(relx=0.83, rely=0.5, relwidth=0.15, relheight=0.4)
    window.mainloop()


if __name__ == "__main__":
    main()
