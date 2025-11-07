import json
import sys

def main():
    name = sys.argv[1]
    lat = sys.argv[2]
    lon = sys.argv[3]
    alt = sys.argv[4]

    json_dict = {"name": name, "latitude": lat, "longitude": lon, "altitude": alt}

    file_name = "C:/Users/maxim/Documents/Unreal Projects/Projektarbeit_Map_NPC_Fertig/Content/" + name + ".json"

    with open(file_name, "w") as f:
        json.dump(json_dict, f, indent=4)

    print(json_dict)

if __name__ == "__main__":
    main()
