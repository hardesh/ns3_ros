#
rosservice call /dccomms_netsim/add_net_device \
"{devType: 0, iddev: operator, frameId: world, mac: 1, \
  trTimeMean: 16, \
  trTimeSd: 0.0, \
  minPrTime: 0.0,  \
  prTimeIncPerMeter: 0.66666, \
  minPktErrorRate: 0.0, \
  pktErrorRateIncPerMeter: 0.0, \
  maxDistance: 350000, \
  minDistance: 0}" 
rosservice call /dccomms_netsim/add_net_device \
"{devType: 0, iddev: rov, frameId: girona500, mac: 2, \
  trTimeMean: 11.634, \
  trTimeSd: 4.698, \
  minPrTime: 0.0, \
  prTimeIncPerMeter: 0.6666, \
  minPktErrorRate: 0.01, \
  pktErrorRateIncPerMeter: 0.0, \
  maxDistance: 350000, \
  minDistance: 0}"
