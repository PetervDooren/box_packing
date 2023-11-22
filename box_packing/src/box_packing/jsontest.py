import json

import rdflib


def main():
    g = rdflib.Graph()
    file = open("/home/peter/ros/flexcraft/system/src/box_packing/src/box_packing/skills/demo.nt")
    g.parse(file=file)
    print(g.serialize(destination="sanity.jsonld",format="json-ld"))
    print(g)

if __name__ == '__main__':
    main()