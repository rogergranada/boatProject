#!/usr/bin/python
#-*- coding: utf-8 -*-
"""
This script creates a map and adds points where the USV 
recorded frames. Before running the script, make sure that
you have folium and pandas libraries installed. 
Otherwise, install it using:

$ pip install folium
$ pip install pandas

"""
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse
from os.path import join, dirname, splitext, basename, realpath
import folium
import pandas as pd


def main(csv_file, file_output=None, lat='gps_lat', lon='gps_long'):
    """ 
    Receives a CSV file containing latitude and longitude
    coordinates named as `lat` and `lon`, respectively.
    """
    csv_file = realpath(csv_file)
    if not file_output:
        dirout = dirname(csv_file)
        fname, _ = splitext(basename(csv_file))
        file_output = join(dirout, fname+'.html')
    file_output = realpath(file_output)
    
    logger.info('Reading CSV file: %s' % csv_file)
    df = pd.read_csv(csv_file, sep=';')
    if lat not in df.columns:
        logger.error('Input file missing `lat` column' % lat)
        sys.exit(0) 
    if lon not in df.columns:
        logger.error('Input file missing `%s` column' % lon)
        sys.exit(0)

    map_points = folium.Map([df[lat].mean(), df[lon].mean()], zoom_start=18)
    for row in df[[lat, lon]].values:
        folium.CircleMarker(location=[row[0], row[1]],
                        fill_color='#3186cc',
                        radius=2,
                        weight=0).add_to(map_points)
    map_points.fit_bounds(map_points.get_bounds())
    map_points.save(file_output)
    logger.info('Saving HTML file: %s' % file_output)


if __name__== "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('fileinput', metavar='csv_file', 
                        help='CSV file containing latitude and longitude coordinates.')
    parser.add_argument('-o', '--output', default=None, 
                        help='Output file to save the HTML map.')
    args = parser.parse_args()
    
    main(args.fileinput, args.output)
