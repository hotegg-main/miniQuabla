import os

def make_result_directory(path_src, name, mode):

    _dir_result = path_src + os.sep + 'result_' + name + '_' + mode
    if os.path.exists(_dir_result):
        resultdir_org = _dir_result
        i = 1
        while os.path.exists(_dir_result):
            _dir_result = resultdir_org + '_%02d' % (i)
            i += 1
    
    os.mkdir(_dir_result)

    return _dir_result