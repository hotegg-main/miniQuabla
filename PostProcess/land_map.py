'''
落下分散図をkmlに出力
'''
import os
import matplotlib.cm as cm
import numpy as np
import simplekml
import pandas as pd

def plot_kml(path, launch_LLH, mag_dec, pos_hard, pos_soft, wind_array, azimuth_array):

    col = len(pos_hard)
    row = len(pos_hard[0])

    llh_hard = []
    llh_soft = []
    speed_list   = []
    azimuth_list = []
    
    llh_Kml_hard = np.zeros((col, row + 1, 3))
    llh_Kml_soft = np.zeros((col, row + 1, 3))

    for i in range(col):
        for j in range(row):
            
            llh_hard.append(NED2LLH(launch_LLH, pos_hard[i][j], mag_dec))
            llh_soft.append(NED2LLH(launch_LLH, pos_soft[i][j], mag_dec))
            speed_list.append(wind_array[i])
            azimuth_list.append(azimuth_array[j])
            
            llh_Kml_hard[i][j] = NED2LLHforKml(launch_LLH, pos_hard[i][j], mag_dec)
            llh_Kml_soft[i][j] = NED2LLHforKml(launch_LLH, pos_soft[i][j], mag_dec)

        llh_Kml_hard[i][row] = llh_Kml_hard[i][0]
        llh_Kml_soft[i][row] = llh_Kml_soft[i][0]
    
    __output_kml(path, llh_Kml_hard, llh_Kml_soft, wind_array)

    llh_hard = np.array(llh_hard).T
    llh_soft = np.array(llh_soft).T

    __output_csv(path, speed_list, azimuth_list, llh_hard, 'trajectory')
    __output_csv(path, speed_list, azimuth_list, llh_soft, 'parachute')

def __output_csv(path, speed, azimuth, llh, name):
    
    pd.DataFrame({
    
        'Wind Speed [m/s]'  : speed,
        'Wind Azimuth [deg]': azimuth,
        'Latitude [deg]'    : llh[0],
        'Longtitude [deg]'  : llh[1],
    
    }).to_csv(path + os.sep + 'land_point_' + name + '.csv', index=False)

def __output_kml(path, hard_LLH, soft_LLH, wind_array):
    
    kml = simplekml.Kml()

    __make_kml_linestring(kml, hard_LLH, wind_array, 'Trajectory', simplekml.Color.orange)
    __make_kml_linestring(kml, soft_LLH, wind_array, 'Parachute' , simplekml.Color.aqua)

    kml.save(path + os.sep + 'land_point' + '.kml')
    
def __make_kml_linestring(kml, pos_LLH, wind_array, name, color):
    
    col = len(pos_LLH)
    lines_hard = [kml.newlinestring(name=str(wind) + 'm/s, ' + name,
                                    coords=hard, 
                                    altitudemode=simplekml.AltitudeMode.relativetoground, 
                                    ) for wind, hard in zip(wind_array, pos_LLH)]
    
    for i in range(col):
        lines_hard[i].style.linestyle.color = color
        lines_hard[i].style.linestyle.width = 4

if __name__=='__main__':

    from coordinate import NED2LLH, NED2LLHforKml

else:
    from .coordinate import NED2LLH, NED2LLHforKml
    