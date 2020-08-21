#!/usr/bin/env python

if __name__ == '__main__':
    """
    Fixes bug with vectors assignment in ArUco library
    """

    FILE_NAME = 'aruco-3.1.12/src/markerdetector_impl.cpp'
    with open(FILE_NAME, 'r') as f:
        lines = f.readlines()
    with open(FILE_NAME, 'w+') as f:
        for line in lines:
            if line.strip() != 'MarkerCanditates[i]=corners[i];':
                f.write(line)
            else:
                f.write(
                    '    {                                                     '
                    '        MarkerCanditates[i].resize(corners[i].size());    '
                    '        for (int j = 0; j < corners[i].size(); j++) {     '
                    '            MarkerCanditates[i][j] = corners[i][j];       '
                    '        }                                                 '
                    '    }                                                     ')
