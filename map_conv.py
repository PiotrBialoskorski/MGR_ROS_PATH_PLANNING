import argparse
from PIL import Image
import cv2
import numpy as np
import os

def conversion(map1): #map1 sciezka do zdjecia wraz z roszerzeniem
    #Sciezka do map
    map_loc_path = r'/home/roslab/ros2_mgr/maps/'
    map = map_loc_path+map1
    #Wczytanie mapy
    map = cv2.imread(map)
    #konwersja do skali szarosci
    gray_map = cv2.cvtColor(map,cv2.COLOR_BGR2GRAY)
    #zapis mapy jako pgm
    ret, gray_map = cv2.threshold(gray_map, 100, 255, cv2.THRESH_BINARY)
    pgm_map = map1.replace(".png", ".pgm")
    os.chdir(map_loc_path)
    cv2.imwrite(pgm_map, gray_map)

def map_yaml(map1):
    filename = map1.replace(".png", ".yaml")
    os.chdir("/home/roslab/mgr/maps/")
    os.remove(filename)
    file = open(filename, "x")
    file.write("image: ")
    file.write(map1.replace(".png", ".pgm"))
    file.write("\n")
    file.write("resolution: 0.05\n")
    file.write("origin: [0.0, 0.0, 0.0]\n")
    file.write("negate: 0\n")
    file.write("occupied_thresh: 0.65\n")
    file.write("free_thresh: 0.196\n")
    file.close()

def main():
    parser = argparse.ArgumentParser(description="Skrypt do konwertowania map z dowolnego formatu na format pgm")

    parser.add_argument("Map1", type=str, help="Nazwa mapy do konwersji")

    args = parser.parse_args()

    conversion(args.Map1)
    map_yaml(args.Map1)

if __name__ == "__main__":
    main()