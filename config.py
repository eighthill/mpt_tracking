import dummy
import nofilter
from kalman import KalmanFilter, KalmanFilterRandomNoise, KalmanFilterAngular
from Kalman_CV2 import ConstantVelocity2, ConstantVelocity
#from CV2 import ConstantVelocity2

# TODO: Add your filters here
filters = {
    "Dummy": {
        "color": [0.2, 0.2, 0.6],
        "constantposition": dummy.DummyFilter(2),
        "constantvelocity": dummy.DummyFilter(2),
        "constantvelocity2": dummy.DummyFilter(2),
        "constantturn": dummy.DummyFilter(2),
        "randomnoise": dummy.DummyFilter(2),
        "angular": dummy.DummyFilter(2),
    }, 
    "NoFilter": {
    "color": [0.5, 0.2, 1.0],
    "constantposition": nofilter.NoFilter(),
    "constantvelocity": nofilter.NoFilter(),
    "constantvelocity2": nofilter.NoFilter(),
    "constantturn": nofilter.NoFilter(),
    "randomnoise": nofilter.NoFilter(),
    "angular": nofilter.NoFilter(),
  },
  "MeMaMa": {
    "color": [1.0, 0.2, 0.2],
    "constantposition": KalmanFilter((2,)),
    "randomnoise": KalmanFilterRandomNoise(2,),
    "angular": KalmanFilterAngular(),
    "constantvelocity": ConstantVelocity(),
    "constantvelocity2" : ConstantVelocity2()
    }
}
