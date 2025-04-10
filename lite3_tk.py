import socket
import struct
import tkinter as tk

class RobotCommander:
    def __init__(self, local_port=20001, ctrl_ip="192.168.1.120", ctrl_port=43893):
        self.local_port = local_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.root = tk.Tk()
        self.root.title("lite3 ctrl")
        self.current_key_label = tk.Label(self.root, text="cur key:")
        self.create_buttons()
        self.bind_keyboard()

    def create_buttons(self):
        button_data = [
            ("Basic Status", [
                ("Heartbeat (k)", 0x21040001),
                ("Reset (X)", 0x21010C05),
                ("Stand Up / Lie Down (Z)", 0x21010202),
                ("Stationary Mode", 0x21010D05),
                ("Movement Mode (C)", 0x21010D06),
            ]),
            ("Gait", [
                ("Low Speed", 0x21010300),
                ("Medium Speed (R)", 0x21010307),
                ("High Speed (T)", 0x21010303),
                ("Normal / Crawling (Y)", 0x21010406),
                ("Ground Gripping (U)", 0x21010402),
                ("Obstacle Crossing (I)", 0x21010401),
                ("High Stepping (V)", 0x21010407),

            ]),
            ("Actions", [
                ("Twist Body", 0x21010204),
                ("Roll Over", 0x21010205),
                ("Moonwalk", 0x2101030C),
                ("Backflip", 0x21010502),
                ("Wave Hello", 0x21010507),
                ("Jump Forward", 0x2101050B),
                ("Twist Jump", 0x2101020D),
            ]),
            ("Movement", [
                ("Move Forward (W)", 0x21010130, 15000, 0),
                ("Move Backward (S)", 0x21010130, -15000, 0),
                ("Move Left (A)", 0x21010131, -30000, 0),
                ("Move Right (D)", 0x21010131, 30000, 0),
                ("Turn Left (Q)", 0x21010135, -32000, 0),
                ("Turn Right (E)", 0x21010135, 32000, 0),
            ]),
            ("Mode Switching", [
                ("Manual Mode (B)", 0x21010C02),
                ("Navigation Mode (N)", 0x21010C03),
                ("Soft Emergency Stop (P)", 0x21010C0E),
                ("Save Data!!!", 0x21010C01)
            ])
        ]

        button_font = ("黑体", 11)
        button_bg_color = "#FFFFFF"
        button_active_bg_color = "#FFFACD"

        for group_row, (group_name, group_buttons) in enumerate(button_data):
            group_frame = tk.LabelFrame(self.root, text=group_name, font=button_font)
            group_frame.grid(row=group_row, column=0, padx=10, pady=4, sticky="nsew")

            for button_col, (text, command, *params) in enumerate(group_buttons):
                button = tk.Button(
                    group_frame,
                    text=text,
                    font=button_font,
                    bg=button_bg_color,
                    activebackground=button_active_bg_color,
                    command=lambda cmd=command, p=params: self.send_command(cmd, *p)
                )
                button.grid(row=0, column=button_col, pady=5, padx=4, sticky="nsew")
                button.config(fg='black')
        for col in range(len(button_data[0][1])):
            self.root.grid_columnconfigure(col, weight=1)

        for row in range(len(button_data)):
            self.root.grid_rowconfigure(row, weight=1)

        self.current_key_label.grid(row=len(button_data), column=0, pady=10)

    def bind_keyboard(self):
        self.root.bind("<KeyPress>", self.handle_key_press)
        self.root.bind("<KeyRelease>", self.handle_key_release)

    def handle_key_press(self, event):
        key_to_command = {
            'z': (0x21010202, 0),      
            'x': (0x21010C05, 0),      
            'c': (0x21010D06, 0),      
            'w': (0x21010130, 32767),    # 前进
            's': (0x21010130, -32767),   # 后退
            'a': (0x21010131, -32767),   # 左平移
            'd': (0x21010131, 32767),    # 右平移
            'q': (0x21010135, -32767),   # 左转
            'e': (0x21010135, 32767),   # 右转
            'r': (0x21010307, 0),       
            't': (0x21010303, 0),       
            'y': (0x21010406, 0),       
            'u': (0x21010402, 0),       
            'i': (0x21010401, 0),       
            'v': (0x21010407, 0),       
            'b': (0x21010C02, 0),        #手动
            'n': (0x21010C03, 0),        #导航
            'p': (0x21010C0E, 0),        #软急停
        }
        key = event.char.lower()
        if key in key_to_command:
            code, param = key_to_command[key]
            self.send_command(code, param)
            self.update_current_key_label(key)

    def handle_key_release(self, event):
        key_to_command = {
            'w': (0x21010130, 0),      # 停止前进
            's': (0x21010130, 0),      # 停止后退
            'a': (0x21010131, 0),      # 停止左平移
            'd': (0x21010131, 0),      # 停止右平移
            'q': (0x21010135, 0),      # 停止左转
            'e': (0x21010135, 0),      # 停止右转
        }
        key = event.char.lower()
        if key in key_to_command:
            code, param = key_to_command[key]
            self.send_command(code, param)
            self.update_current_key_label(key)


    def continuous_command(self, code, param1=0, param2=0):
        self.send_simple(code, param1, param2)
        self.after_id = self.root.after(100, self.continuous_command, code, param1, param2)

    def send_command(self, code, param1=0, param2=0):
        print(f"Sending command={code}, Param1={param1}, Param2={param2}")
        self.send_simple(code, param1, param2)

    def send_simple(self, code, param1=0, param2=0):
        try:
            payload = struct.pack('<3i', code, param1, param2)
            self.server.sendto(payload, self.ctrl_addr)
        except Exception as e:
            print(f"Error sending command:：{e}")

    def update_current_key_label(self, key):
        self.current_key_label.config(text=f"Current button: {key.upper()}")

    def start_heartbeat(self):
        self.continuous_command(0x21040001)

    def stop_continuous_command(self):
        if hasattr(self, 'after_id'):
            self.root.after_cancel(self.after_id)

    def on_closing(self):
        self.stop_continuous_command()
        self.server.close()
        self.root.destroy()

    def run(self):
        self.start_heartbeat() 
        self.root.mainloop()

if __name__ == "__main__":
    gui = RobotCommander()
    gui.run()