import csv

import matplotlib.pyplot as plt

if __name__ == "__main__":
    data = []
    gt_data = []
    with open('/home/magnus/CAS/SensorFusion/data.csv') as csvfile:
        data_tmp = csv.reader(csvfile)
        for row in data_tmp:
            data.append(row)

    with open('/home/magnus/CAS/SensorFusion/gps_data_converted.csv') as csvfile:
        data_tmp = csv.reader(csvfile)
        for row in data_tmp:
            gt_data.append(row)


    fig, ax = plt.subplots()
    e = [float(i[0]) - float(data[0][0]) for i in data]
    n = [float(i[1]) - float(data[0][1]) for i in data]
    e1 = [float(i[0]) - float(gt_data[0][0]) for i in gt_data]
    n1 = [float(i[1]) - float(gt_data[0][1]) for i in gt_data]
    ax.plot(e, n, label='Path')
    ax.plot(e1, n1, label='GT')
    legend = ax.legend(loc='best', shadow=True, fontsize='medium')
    plt.show()
    
