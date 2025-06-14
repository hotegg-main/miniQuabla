'''
落下分散図をkmlに出力
'''
import os
import matplotlib.cm as cm
import numpy as np
import simplekml

def plot_kml(path, launch_LLH, mag_dec, pos_hard, pos_soft, wind_array):

    col = len(pos_hard)
    row = len(pos_hard[0])

    llh_hard = np.zeros((col, row, 3))
    llh_soft = np.zeros((col, row, 3))
    llh_Kml_hard = np.zeros((col, row, 3))
    llh_Kml_soft = np.zeros((col, row, 3))

    for i in range(col):
        for j in range(row):
            llh_hard[i][j] = NED2LLH(launch_LLH, pos_hard[i][j], mag_dec)
            llh_soft[i][j] = NED2LLH(launch_LLH, pos_soft[i][j], mag_dec)
            
            llh_Kml_hard[i][j] = NED2LLHforKml(launch_LLH, pos_hard[i][j], mag_dec)
            llh_Kml_soft[i][j] = NED2LLHforKml(launch_LLH, pos_soft[i][j], mag_dec)
    
    __output_kml(path, llh_Kml_hard, llh_Kml_soft, wind_array)

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
    