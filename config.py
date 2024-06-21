import dummy
from NoFilter import NoFilter

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
     "No Filter":{
        "color": [0.5, 0.2, 1.0],
        "constantposition": NoFilter(),
         "constantvelocity":  NoFilter(),
         "constantvelocity2":  NoFilter(),
         "constantturn":  NoFilter(),
         "randomnoise":  NoFilter(),
         "angular":  NoFilter(),
     }
}
