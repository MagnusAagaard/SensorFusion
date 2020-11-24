import csv

import matplotlib.pyplot as plt

if __name__ == "__main__":
    data = []
    gt_data = []
    noise_data = []
    with open('/home/magnus/CAS/SensorFusion/data.csv') as csvfile:
        data_tmp = csv.reader(csvfile)
        for row in data_tmp:
            data.append(row)

    with open('/home/magnus/CAS/SensorFusion/gps_gt_converted.csv') as csvfile:
        data_tmp = csv.reader(csvfile)
        for row in data_tmp:
            gt_data.append(row)

    with open('/home/magnus/CAS/SensorFusion/gps_noise_converted.csv') as csvfile:
        data_tmp = csv.reader(csvfile)
        for row in data_tmp:
            noise_data.append(row)

    print(len(noise_data))
    fig, ax = plt.subplots()
    e = [float(i[1]) for i in data]
    n = [float(i[0]) for i in data]
    e1 = [float(i[1]) for i in gt_data]
    n1 = [float(i[0]) for i in gt_data]
    #e2 = [float(i[1]) for i in noise_data]
    #n2 = [float(i[0]) for i in noise_data]
    ax.plot(e, n, label='Path')
    ax.plot(e1, n1, label='GPS')
    #ax.plot(e2, n2, label='Noise')
    ax.set(xlabel='East (m)', ylabel='North (m)', title='Drone position - IMU data only')
    legend = ax.legend(loc='best', shadow=True, fontsize='medium')
    plt.show()
    
