'''
落下分散図をkmlに出力
'''
import os
import matplotlib.cm as cm
import numpy as np
import simplekml
import pandas as pd

def output_land_map(path, name, launch_LLH, pos_NED, wind_array, azimuth_array, color):

    col = len(pos_NED)
    row = len(pos_NED[0])

    pos_llh = []
    speed_list   = []
    azimuth_list = []
    
    pos_llh_kml = np.zeros((col, row + 1, 3))

    for i in range(col):
        for j in range(row):
            
            pos_llh.append(NED2LLH(launch_LLH, pos_NED[i][j]))
            speed_list.append(wind_array[i])
            azimuth_list.append(azimuth_array[j])
            
            pos_llh_kml[i][j] = NED2LLHforKml(launch_LLH, pos_NED[i][j])

        pos_llh_kml[i][row] = pos_llh_kml[i][0]

    # CSV出力
    pos_llh = np.array(pos_llh).T
    __output_csv(path, speed_list, azimuth_list, pos_llh, name)

    # KML出力
    kml = simplekml.Kml()
    __make_kml_linestring(kml, pos_llh_kml, wind_array, color)
    kml.save(path + os.sep + 'land_map_' + name + '.kml')

def __output_csv(path, speed, azimuth, llh, name):
    
    pd.DataFrame({
    
        'Wind Speed [m/s]'  : speed,
        'Wind Azimuth [deg]': azimuth,
        'Latitude [deg]'    : llh[0],
        'Longtitude [deg]'  : llh[1],
    
    }).to_csv(path + os.sep + 'land_map_' + name + '.csv', index=False)

def __make_kml_linestring(kml, pos_LLH, wind_array, color):
    
    col = len(pos_LLH)
    lines_hard = [kml.newlinestring(name=str(wind) + 'm/s',
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
    