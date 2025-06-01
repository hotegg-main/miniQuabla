'''
落下分散図をkmlに出力
'''
import os
import matplotlib.cm as cm
import numpy as np

def plot_kml(path, launch_LLH, pos_hard, pos_soft, wind_array):

    col = len(pos_hard)
    row = len(pos_hard[0])

    llh_hard = np.zeros((col, row, 3))
    llh_soft = np.zeros((col, row, 3))
    llh_Kml_hard = np.zeros((col, row, 3))
    llh_Kml_soft = np.zeros((col, row, 3))

    for i in range(col):
        for j in range(row):    
            llh_hard[i][j] = NED2LLH(launch_LLH, pos_hard[i][j])
            llh_soft[i][j] = NED2LLH(launch_LLH, pos_soft[i][j])
            
            llh_Kml_hard[i][j] = NED2LLHforKml(launch_LLH, pos_hard[i][j])
            llh_Kml_soft[i][j] = NED2LLHforKml(launch_LLH, pos_soft[i][j])
    
    __output_kml(path, llh_Kml_hard, llh_Kml_soft, wind_array, cm.cool)

def __output_kml(path, hard_LLH, soft_LLH, wind_array, color_cm):
    import simplekml
    
    kml = simplekml.Kml()
    
    i = 0
    col = len(hard_LLH)
    for hard in hard_LLH:
        
        linestring = kml.newlinestring(name=str(wind_array[i]))
        r = int(color_cm( i / col )[0] * 255)
        g = int(color_cm( i / col )[1] * 255)
        b = int(color_cm( i / col )[2] * 255)
        # linestring.style.linestyle.color = simplekml.Color.rgb(r, g, b)
        linestring.style.linestyle.color = simplekml.Color.orange
        linestring.style.linestyle.width = 2
        linestring.altitudemode = simplekml.AltitudeMode.relativetoground
        linestring.coords = hard
        i += 1
    
    i = 0
    for hard in soft_LLH:
        
        linestring = kml.newlinestring(name=str(wind_array[i]))
        r = int(color_cm( i / col )[0] * 255)
        g = int(color_cm( i / col )[1] * 255)
        b = int(color_cm( i / col )[2] * 255)
        # linestring.style.linestyle.color = simplekml.Color.rgb(r, g, b)
        linestring.style.linestyle.color = simplekml.Color.aqua
        linestring.style.linestyle.width = 2
        linestring.altitudemode = simplekml.AltitudeMode.relativetoground
        linestring.coords = hard
        i += 1

    kml.save(path + os.sep + 'land_point' + '.kml')

if __name__=='__main__':

    from coordinate import NED2LLH, NED2LLHforKml

else:
    from .coordinate import NED2LLH, NED2LLHforKml