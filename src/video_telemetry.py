import numpy as np
import cv2
#import px4tools
import pandas as pd
import pyulog
import math


log = pyulog.ULog("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/2019-05-22/18_22_29.ulg")
print(log.Data)
log.last_timestamp
log.start_timestamp
data = log.data_list
#datalist =data.name

#for x in data:
#    print(x.name)

#pdd = pd.DataFrame.from_dict(log.get_dataset('sensor_preflight').data)
#pdd

#np.min(np.abs(pdd.timestamp - 366465760))

#pdd.iloc[np.min(np.abs(pdd.timestamp - 366875760))]


#print(pdd)

#log.get_dataset('airspeed').data['timestamp']
#print(log.get_dataset('vehicle_command').list_value_changes("param1"))


#log = px4tools.ulog.read_ulog("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/2019-05-22/18_22_29.ulg")
#print(log.position_setpoint_s)
#print(log.keys())

class video_telemetry(object):
    """docstring for video_telemetry."""
    def __init__(self):
        super(video_telemetry, self).__init__()

        self.video_file = None
        self.log_file = None
        self.video_start_time = 0

    def set_video(self, video_file, start_time = 0, skip = 0):
        """
            video_file: filepath for source video_out
            start_time: time of log on begining of video
        """
        self.video_file = video_file
        self.video_start_time = start_time + skip
        self.video_skip = skip

    def out_video(self, video_out):
        self.video_out = video_out

    def set_log(self, log_file):
        self.log_file = log_file

    def load_log(self):
        self.log = pyulog.ULog(self.log_file)
        self.log_start = self.log.start_timestamp
        self.log_keys = [x.name for x in data]

        print("start timestam: ", self.log_start)
        print(self.log_keys)

    def learn(self):
        self.log_start = self.get_table('input_rc').timestamp[0]
        print("start timestam: ", self.log_start)

        table = self.get_table('input_rc')

    def start(self):
        cap = cv2.VideoCapture(self.video_file)
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        print("FPS:", self.fps)
        print("Skip frames:", self.video_skip/1000*self.fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, self.video_skip/1000*self.fps)

        fourcc = cv2.VideoWriter_fourcc(*'X264')

        out = cv2.VideoWriter(self.video_out, fourcc, self.fps, (1920,1080))

        font = cv2.FONT_HERSHEY_SIMPLEX
        self.font = cv2.FONT_HERSHEY_SIMPLEX


        #Ziskani pocatecni vysky
        basealt = self.get_table('vehicle_air_data')[:10]['baro_alt_meter'].mean()

        #print(self.get_table("ekf2_timestamps"))

        while(cap.isOpened()):
            ret, frame = cap.read()

            if ret==True:

                #img = frame.copy()
                self.img = cv2.resize(frame, (1920, 1080), interpolation = cv2.INTER_LINEAR)


                videotime = cap.get(cv2.CAP_PROP_POS_FRAMES)/self.fps*1000
                print(videotime)
                time = videotime + self.video_start_time
                cv2.putText(self.img,'V: {:.2f} s, L: {:.2f} s'.format(videotime/1000, time/1000),(10,60), font, 1,(255,255,255), 1, cv2.LINE_AA)


                spd = self.get_value("airspeed", time)

                self.draw_range((820, 30), spd.true_airspeed_m_s, min = 0, max=30, label = "TAS", value_text="m/s")

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

                self.draw_range((1080, 150), roll, data_sp['roll_body']*57.2957, min = -60, max = 60, label = 'roll', value_text = "d")
                self.draw_range((1000, 150), pitch,data_sp['pitch_body']*57.2957, min = -60, max = 60, label = 'pitch', value_text = "d")
                self.draw_range((1160, 150), yaw, min = -180, max = 180, label = 'yaw', value_text = "d")

                self.draw_range((1080, 30), data_sp['roll_body']*57.2957, min = -45, max = 45, label = 'roll', value_text = "d")
                self.draw_range((1000, 30), data_sp['pitch_body']*57.2957, min = -45, max = 45, label = 'pitch', value_text = "d")
                self.draw_range((920, 30), data_sp['thrust_body[0]']*100, min = 0, max = 100, label = 'throttle', value_text = "%")

                self.draw_inclinometer((100, 300), (roll, pitch, yaw), (data_sp['roll_body']*57.2957, data_sp['pitch_body']*57.2957, data_sp['yaw_body']*57.2957))

                out.write(self.img)
                cv2.imshow('frame',self.img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

        # Release everything if job is finished
        cap.release()
        out.release()
        cv2.destroyAllWindows()

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

if __name__ == '__main__':
    vt = video_telemetry()
    vt.set_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/Videa/C0081.MP4", -21200, skip = 20000)
    #vt.set_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/Videa/C0081.MP4", 20000)
    vt.out_video("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/telem_#25.mp4")
    vt.set_log("/home/roman/OwnCloudMLAB/TF_data/UAV/Auto-G2/zkousky/Test_20190522/2019-05-22/18_22_29.ulg")
    vt.load_log()
    #vt.learn()
    #print(vt.get_table('input_rc').keys())

    vt.start()
