import numpy as np
import cv2
#import px4tools
import pandas as pd
import pyulog
import math
import pygame
import sys



class video_telemetry(object):
    def __init__(self):
        super(video_telemetry, self).__init__()

        self.video_file = None
        self.log_file = None
        self.video_start_time = 0

    def set_video(self, video_file, skip = 0):
        """
            video_file: filepath for source video_out
            start_time: time of log on begining of video
        """
        print("Video vstup:", video_file)
        self.video_file = video_file
        self.video_skip = skip

    def out_video(self, video_out):
        self.video_out = video_out

    def set_log(self, log_file, log_offset = 0):
        print("LOG:", log_file)
        self.log_offset = log_offset
        self.log_file = log_file

    def load_log(self):
        print("\nLoad logs")
        self.log = pyulog.ULog(self.log_file)
        self.log_start = self.log.start_timestamp
        self.log_keys = [x.name for x in self.log.data_list]

        print("start timestam: ", self.log_start)
        #print(self.log_keys)

    def learn(self):
        self.log_start = self.get_table('input_rc').timestamp[0]
        print("start timestam: ", self.log_start)

        table = self.get_table('input_rc')

    def start(self):
        print("\n Start")
        GREEN = (0, 255, 0)
        cap = cv2.VideoCapture(self.video_file)
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        print("FPS:", self.fps)
        print("Skip frames:", self.video_skip/1000*self.fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, self.video_skip/1000*self.fps)

        fourcc = cv2.VideoWriter_fourcc(*'X264')

        out = cv2.VideoWriter(self.video_out, fourcc, self.fps, (1920,1080))

        font = cv2.FONT_HERSHEY_SIMPLEX
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        pygame.init()
        pygame.display.set_caption('Nahled')
        display_surface = pygame.display.set_mode((1920, 1080))

        #Ziskani pocatecni vysky
        basealt = self.get_table('vehicle_air_data')[:10]['baro_alt_meter'].mean()

        #print(self.get_table("ekf2_timestamps"))
        basic_font = pygame.font.Font(None, 30)

        # Nakresleni pomocnych car..
        hud_background = pygame.Surface((1920, 1080), pygame.SRCALPHA)
        pygame.draw.arc(hud_background, GREEN, (1920//2-200, 200, 400, 400), math.radians(30), math.radians(150), 3)

        for a in [-60, -30, 0, 30, 60]:
            x = math.cos(math.radians(a+90))
            y = math.sin(math.radians(a+90))
            pygame.draw.line(hud_background, GREEN, (1920/2 + 200*x, 400-200*y), (1920/2 + 215*x, 400-215*y), 3)

        for a in [-45, -10, -20, 10, 20, 45]:
            x = math.cos(math.radians(a+90))
            y = math.sin(math.radians(a+90))
            pygame.draw.line(hud_background, GREEN, (1920/2 + 200*x, 400-200*y), (1920/2 + 210*x, 400-210*y), 2)

        pygame.draw.line(hud_background, GREEN, [860, 540], [1060, 540], 1)
        pygame.draw.line(hud_background, GREEN, [960, 440], [960, 640], 1)

        running = True
        while(cap.isOpened() and running):
            ret, frame = cap.read()

            if ret==True:

                #img = frame.copy()
                self.img = cv2.resize(frame, (1920, 1080), interpolation = cv2.INTER_LINEAR)

                videotime = cap.get(cv2.CAP_PROP_POS_FRAMES)/self.fps*1000
                time = videotime + self.log_offset
                cv2.putText(self.img,'V: {:.2f} s, L: {:.2f} s'.format(videotime/1000, time/1000),(10,60), font, 1,(255,255,255), 1, cv2.LINE_AA)


                spd = self.get_value("airspeed", time)
                self.draw_range((820, 30), spd.true_airspeed_m_s, min = 0, max=30, label = "TAS", value_text="ms")

                data = self.get_value("vehicle_gps_position", time)
                self.draw_range((890, 30), data.vel_m_s, min = 0, max=30, label = "GSpd", value_text="ms")


                data = self.get_value("manual_control_setpoint", time)
                self.draw_RC(data)

                data = self.get_value('vehicle_air_data', time)
                #self.draw_range((300, 100), data['baro_alt_meter'], min = basealt, max = basealt+100, label = 'Alt', value_text = "m")
                self.draw_range((750, 30), data['baro_alt_meter']-basealt, min = 0, max = 100, label = 'rAlt', value_text = "m")

                # Baterie ..
                spd = self.get_value("battery_status", time)
                self.draw_range((450, 30), spd['voltage_filtered_v'], min = 0, max = 15, label = 'batU', value_text = "V")
                self.draw_range((530, 30), spd['current_filtered_a'], min = 0, max = 30, label = 'batI', value_text = "A")

                # Akcelorometr
                spd = self.get_value("sensor_combined", time)
                self.draw_range((680, 30), spd['accelerometer_m_s2[2]']/9.8, min = -5, max = 5, label = 'acc-Z', value_text = "g")


                data = self.get_value("vehicle_attitude", time)
                data_sp=self.get_value("vehicle_attitude_setpoint", time)

                d = [data['q[0]'], data['q[1]'], data['q[2]'], data['q[3]']]
                (roll, pitch, yaw) = self.get_roll_pitch_yaw(d)

                self.draw_range((1080, 30), roll, data_sp['roll_body']*57.2957, min = -45, max = 45, label = 'roll', value_text = "d")
                self.draw_range((1000, 30), pitch,data_sp['pitch_body']*57.2957, min = -45, max = 45, label = 'pitch', value_text = "d")
                self.draw_range((1160, 30), yaw, min = -180, max = 180, label = 'yaw', value_text = "d")

                self.draw_inclinometer((100, 300), (roll, pitch, yaw), (data_sp['roll_body']*57.2957, data_sp['pitch_body']*57.2957, data_sp['yaw_body']*57.2957))

                self.pimg = pygame.image.frombuffer(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB).tostring(), self.img.shape[1::-1], "RGB")

                hud = pygame.Surface((1000, 800), pygame.SRCALPHA)

                round_pitch = 5 * round(pitch/5)
                for i in range(round_pitch-20, round_pitch+21, 5):
                    offset = pitch-i
                    offset_x = offset * 10
                    text = basic_font.render("{}".format(i), True, (0, 255, 0))
                    texto = basic_font.render("{}".format(offset), True, (0, 255, 0))

                    if i == 0:
                        pygame.draw.lines(hud, GREEN, False, [[300, 400+offset_x], [490, 400+offset_x], [500, 420+offset_x], [510, 400+offset_x], [700, 400+offset_x]], 3)
                    elif i % 10 == 0:
                        pygame.draw.line(hud, GREEN, [350, 400+offset_x], [475, 400+offset_x], 2)
                        pygame.draw.line(hud, GREEN, [525, 400+offset_x], [650, 400+offset_x], 2)
                        hud.blit(text, (290-text.get_width(),400+offset_x-10))
                        hud.blit(text, (710,400+offset_x-10))
                    elif i % 5 == 0:
                        pygame.draw.line(hud, GREEN, [375, 400+offset_x], [475, 400+offset_x], 2)
                        pygame.draw.line(hud, GREEN, [525, 400+offset_x], [626, 400+offset_x], 2)
                        hud.blit(text, (340-text.get_width(),400+offset_x-10))
                        hud.blit(text, (660,400+offset_x-10))
                pygame.draw.circle(hud, GREEN, [500, 400], 5)

                hud = pygame.transform.rotate(hud, roll)
                (w, h) = hud.get_size()
                self.pimg.blit(hud, (1920//2-w//2, 1080//2-h//2))
                self.pimg.blit(hud_background, (0, 0))

                hud = pygame.Surface((1000, 800), pygame.SRCALPHA)
                pygame.draw.lines(hud, GREEN, True, ([500, 200-2], [495, 180], [505, 180]), 3)
                hud = pygame.transform.rotate(hud, roll)
                (w, h) = hud.get_size()
                self.pimg.blit(hud, (960-w/2, 400-h/2))

                hud = pygame.Surface((1000, 800), pygame.SRCALPHA)
                pygame.draw.lines(hud, (255, 255, 0), True, ([500, 200+2], [495, 220], [505, 220]), 3)
                hud = pygame.transform.rotate(hud, math.degrees(data_sp['roll_body']))
                (w, h) = hud.get_size()
                self.pimg.blit(hud, (960-w/2, 400-h/2))

                view = pygame.surfarray.array3d(self.pimg)
                view = view.transpose([1, 0, 2])
                self.img = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
                out.write(self.img)

                #cv2.imshow('frame',self.img)
                display_surface.blit(self.pimg, (0, 0))

                for event in pygame.event.get():
                    if event.type == pygame.QUIT or event.type == pygame.KEYDOWN:
                        running = False

                pygame.display.update()

            else:
                break

        # Release everything if job is finished
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        pygame.quit()

    def draw_range(self, pos, value = 0.5, value_sp = None, min=0, max=1, size = (15, 70), label = None, value_text = None):
        p2 = (pos[0]+ size[0], pos[1]+size[1])
        cv2.rectangle(self.img,pos, p2, (0,0,10, 100),-1)
        cv2.rectangle(self.img,pos, p2, (0,255,0),1)

        shift = int((1-(value-min)/abs(min-max))*size[1])

        p1 = (pos[0]+2, pos[1]+2 + shift)
        p2 = (pos[0]+ size[0]-2, pos[1]+size[1]-2)
        cv2.rectangle(self.img, p1, p2, (255,255,0),-1)


        if value_sp is not None:
            shift_sp = int((1-(value_sp-min)/abs(min-max))*size[1])
            p1 = (pos[0]-5, pos[1]+2 + shift_sp)
            p2 = (pos[0]+5+size[0], pos[1]+2 + shift_sp)
            cv2.line(self.img, p1, p2, (0, 255, 255), 3)
            if value_text:
                cv2.putText(self.img, "{:.2f}{}".format(value_sp, value_text), (pos[0]-15, pos[1] + size[1] + 30), 1, 1,(255,255,255), 1, cv2.LINE_AA)

        if label:
            cv2.putText(self.img, label, (pos[0]-15, pos[1]-5), 1, 1,(255,255,255), 1, cv2.LINE_AA)

        if value_text:
            cv2.putText(self.img, "{:.2f}{}".format(value, value_text), (pos[0]-15, pos[1] + size[1] + 15), 1, 1,(255,255,255), 1, cv2.LINE_AA)

    def get_roll_pitch_yaw(self, q):

        roll = np.arctan2(2.0 * (q[0] * q[1] + q[2] * q[3]),
                            1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
        pitch = np.arcsin(2.0 * (q[0] * q[2] - q[3] * q[1]))
        yaw = np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

        return(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def draw_RC(self, manual_control, p1 = (50, 100)):
        p2 = (p1[0]+235, p1[1]+100)

        # celkovy ramecek
        cv2.rectangle(self.img, p1, p2, (100, 100, 100, 20), -1)

        # ramecek pro joysticky
        cv2.rectangle(self.img, (p1[0]+25, p1[1]+5), (p1[0]+115, p1[1]+95), (150, 150, 150, 20), -1)
        cv2.rectangle(self.img, (p1[0]+120, p1[1]+5), (p1[0]+210, p1[1]+95), (150, 150, 150, 20), -1)

        # ramecek pro slidery
        cv2.rectangle(self.img, (p1[0]+5, p1[1]+5), (p1[0]+20, p1[1]+95), (150, 150, 150, 20), -1)
        cv2.rectangle(self.img, (p1[0]+215, p1[1]+5), (p1[0]+230, p1[1]+95), (150, 150, 150, 20), -1)

        cv2.circle(self.img, (int(p1[0]+70 +45*manual_control.r), int(p1[1]+50-45*(manual_control.z-0.5)*2)), 5, (0, 0, 0), -1)
        cv2.circle(self.img, (int(p1[0]+165+45*manual_control.y), int(p1[1]+50-45*manual_control.x)), 5, (0, 0, 0), -1)


    def draw_inclinometer(self, p1, estimated, setpoint = None):
        cv2.circle(self.img, (p1[0]+45, p1[1]+45), 45, (100, 100, 100), -1)
        cv2.circle(self.img, (p1[0]+140, p1[1]+45), 45, (100, 100, 100), -1)

        if setpoint:
            rx = math.cos(math.radians(setpoint[0]))*45
            ry = math.sin(math.radians(setpoint[0]))*45
            px = math.cos(math.radians(setpoint[1]))*45
            py = math.sin(math.radians(setpoint[1]))*45

            cv2.line(self.img, (int(p1[0]+45-rx), int(p1[1]+45-ry)), (int(p1[0]+45+rx), int(p1[1]+45+ry)), (255, 0, 255), 1, cv2.LINE_AA)
            cv2.line(self.img, (int(p1[0]+140), int(p1[1]+45)), (int(p1[0]+140+px), int(p1[1]+45-py)), (255, 0, 255), 1, cv2.LINE_AA)


        rx = math.cos(math.radians(estimated[0]))*45
        ry = math.sin(math.radians(estimated[0]))*45

        px = math.cos(math.radians(estimated[1]))*45
        py = math.sin(math.radians(estimated[1]))*45

        cv2.line(self.img, (int(p1[0]+45-rx), int(p1[1]+45-ry)), (int(p1[0]+45+rx), int(p1[1]+45+ry)), (0, 255, 255), 2, cv2.LINE_AA)
        cv2.line(self.img, (int(p1[0]+140), int(p1[1]+45)), (int(p1[0]+140+px), int(p1[1]+45-py)), (0, 255, 255), 2, cv2.LINE_AA)

        cv2.putText(self.img, "Roll", (p1[0]+43, p1[1]), 1, 1,(255,255,255), 1, cv2.LINE_AA)
        cv2.putText(self.img, "Pitch", (p1[0]+137, p1[1]), 1, 1,(255,255,255), 1, cv2.LINE_AA)



    def get_value(self, table, time = 0):
        tab = self.get_table(table)
        ix = (tab['timestamp'] - (self.log_start + time*1000)).abs().idxmin()
        return tab.iloc[ix]

    def get_table(self, name):
        data = pd.DataFrame.from_dict(self.log.get_dataset(name).data)
        return data

    def load_config(self, cfg):

        cfgj = {
            "log": "2019-06-02/10_49_36.ulg",
            "log_start": 40700,
            "video": "Videa/C0105.MP4",
            "video_start": 29000,
            "video_out": "telem2.MP4"
        }
        url = "/home/roman/OwnCloudMLAB/TF_data/UAV/TF-G1/zkousky/Test_20190602/"

        self.set_video(url+cfgj['video'], cfgj['video_start'])
        self.set_log(url+cfgj['log'], cfgj['log_start'])
        self.load_log()
        self.out_video(url+cfgj['video_out'])

if __name__ == '__main__':

    print("Parametru:", len(sys.argv))
    if True:
        print("Usage: python3 video_telemetry.py <configuration-file>")

    cfg = ""
    vt = video_telemetry()
    vt.load_config(cfg)
    vt.start()

    #vt.set_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/Videa/C0081.MP4", skip = 20000)
    #vt.out_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/telem_#25.mp4")
    #vt.set_log("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/2019-05-22/18_22_29.ulg")

    #vt.set_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190526/Videa/C0093.MP4", skip = 0)
    #vt.out_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190526/telem5.MP4")
    #vt.set_log("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190526/2019-05-26/10_59_48.ulg")

    #vt.set_video("/home/roman/OwnCloudMLAB/TF_data/UAV/TF-G1/zkousky/Test_20190502/Videa/C0105.MP4", skip = 29000)
    #vt.out_video("/home/roman/OwnCloudMLAB/TF_data/UAV/TF-G1/zkousky/Test_20190502/telem.MP4")
    #vt.set_log("/home/roman/OwnCloudMLAB/TF_data/UAV/TF-G1/zkousky/Test_20190502/2019-06-02/10_49_36.ulg")

    # vt.load_config(cfg)
    # vt.start()
