import overpy
import os


_SIM = "SIMULATION" in os.environ
_DEBUG = True


def _debug(msg):
  if not _DEBUG:
    return
  print(msg)


class OSM():
  def __init__(self):
    # self.api = overpy.Overpass()
    self.api = overpy.Overpass(url='http://3.65.170.21/api/interpreter')

  def fetch_road_ways_around_location(self, lat, lon, radius):
    if _SIM:
      return self.simulator_fetch()

    # fetch all ways and nodes on this ways around location
    around_str = f'{str(radius)},{str(lat)},{str(lon)}'
    q = """
        way(around:""" + around_str + """)
          [highway]
          [highway!~"^(footway|path|bridleway|steps|cycleway|construction|bus_guideway|escape|service)$"];
        (._;>;);
        out;
        """
    try:
      ways = self.api.query(q).ways
    except Exception as e:
      print(f'Exception while querying OSM:\n{e}')
      ways = []

    return ways

  def simulator_fetch(self):
    _debug('OSM: Start simulator fetch')

    path = os.getenv("CARLA_OVERPASS_XML_PATH", "../mapd/lib/carla_town04_osm_overpass.xml")

    with open(path, 'r') as f:
      overpass_xml = f.read()
      ways = self.api.parse_xml(overpass_xml).ways

    _debug('OSM: Finish simulator fetch')
    return ways
