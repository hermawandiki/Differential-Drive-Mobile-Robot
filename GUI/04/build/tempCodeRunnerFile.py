canvas.itemconfig(usl_text, text=sensor_data.usl)
    canvas.itemconfig(usf_text, text=sensor_data.usf)
    canvas.itemconfig(usr_text, text=sensor_data.usr)
    canvas.itemconfig(encl_text, text=sensor_data.encl)
    canvas.itemconfig(encr_text, text=sensor_data.encr)
    canvas.itemconfig(pitch_text, text=sensor_data.pitch)
    canvas.itemconfig(roll_text, text=sensor_data.roll)
    canvas.itemconfig(yaw_text, text=sensor_data.yaw)
    window.after(1, update_dashboard)