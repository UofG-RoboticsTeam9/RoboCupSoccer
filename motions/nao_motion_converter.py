#!/usr/bin/env python3

import csv

# You can add your copied lists here like the example.
########################################################################

names = list()
times = list()
keys = list()

names.append("HeadPitch")
times.append([0.04])
keys.append([0])

names.append("HeadYaw")
times.append([0.04])
keys.append([0])

names.append("LAnklePitch")
times.append([0.04])
keys.append([-0.345059])

names.append("LAnkleRoll")
times.append([0.04])
keys.append([-0.00852968])

names.append("LElbowRoll")
times.append([0.04])
keys.append([-1.00904])

names.append("LElbowYaw")
times.append([0.04])
keys.append([-1.3807])

names.append("LHand")
times.append([0.04])
keys.append([0.257812])

names.append("LHipPitch")
times.append([0.04])
keys.append([-0.443486])

names.append("LHipRoll")
times.append([0.04])
keys.append([0.00112307])

names.append("LHipYawPitch")
times.append([0.04])
keys.append([0])

names.append("LKneePitch")
times.append([0.04])
keys.append([0.691128])

names.append("LShoulderPitch")
times.append([0.04])
keys.append([1.40895])

names.append("LShoulderRoll")
times.append([0.04])
keys.append([0.297248])

names.append("LWristYaw")
times.append([0.04])
keys.append([0.00399397])

names.append("RAnklePitch")
times.append([0.04])
keys.append([-0.345059])

names.append("RAnkleRoll")
times.append([0.04])
keys.append([0.00852968])

names.append("RElbowRoll")
times.append([0.04])
keys.append([1.00904])

names.append("RElbowYaw")
times.append([0.04])
keys.append([1.3807])

names.append("RHand")
times.append([0.04])
keys.append([0.257812])

names.append("RHipPitch")
times.append([0.04])
keys.append([-0.443486])

names.append("RHipRoll")
times.append([0.04])
keys.append([-0.00112307])

names.append("RHipYawPitch")
times.append([0.04])
keys.append([0])

names.append("RKneePitch")
times.append([0.04])
keys.append([0.691128])

names.append("RShoulderPitch")
times.append([0.04])
keys.append([1.40895])

names.append("RShoulderRoll")
times.append([0.04])
keys.append([-0.297248])

names.append("RWristYaw")
times.append([0.04])
keys.append([0.00399397])

########################################################################

def convert_time_array(arr):
    out = []
    pose = 1
    for el in arr:
        webots_time = convert_time_to_webots_time(el, pose)
        out.append(webots_time)
        pose += 1

    return out


def convert_time_to_webots_time(time, pose):
    tmp = (time % 1)
    milisec = tmp * 1000
    sec = time - tmp

    return '00:' + ("%02d" % (sec,)) + ':' + ("%03d" % (milisec,)) + ',' + ('Pose%d' % pose)


all_names, all_times, all_keys = names, times, keys
first_row = ['#WEBOTS_MOTION','V1.0'] + all_names

if __name__ == "__main__":
    where_to = "StandInit.motion"
    motion = where_to
    time_set = set()
    with open(motion, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(first_row)

        # Get all times
        for times in all_times:
            for time in times:
                time_set.add(time)

        time_array = sorted(time_set)
        time_array_webots = convert_time_array(time_array)

        for name,times,keys in zip(all_names,all_times,all_keys):
            indexes = []
            for time,key in zip(times,keys):
                index = time_array.index(time)
                indexes.append(index)
                time_array_webots[index] += (',' + str(key))

            time_array_webots = [(x + ',*') if (i not in indexes) else time_array_webots[i] for i,x in enumerate(time_array_webots) ] 
            print(time_array_webots)
                   
        out_csv = []
        for row in time_array_webots:
            out = row.split(',')
            out_csv.append(out)
                
        writer.writerows(out_csv)