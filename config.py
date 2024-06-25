import dummy
from NoFilter import NoFilter
import statObj

# TODO: Add your filters here
filters = {
    "Dummy": {
        "color": [0.2, 0.2, 0.4],
        "constantposition": dummy.DummyFilter(2),
        "constantvelocity": dummy.DummyFilter(2),
        "constantvelocity2": dummy.DummyFilter(2),
        "constantturn": dummy.DummyFilter(2),
        "randomnoise": dummy.DummyFilter(2),
        "angular": dummy.DummyFilter(2),
    },
     "NoFilter":{
        "color": [0.5, 0.2, 1.0],
        "constantposition": NoFilter(),
         "constantvelocity":  NoFilter(),
         "constantvelocity2":  NoFilter(),
         "constantturn":  NoFilter(),
         "randomnoise":  NoFilter(),
         "angular":  NoFilter(),
     },
     "StaticObject":{
        "color": [0.8, 0.2, 0.1],
        "constantposition": statObj.staticFilter(2)
     }
}
