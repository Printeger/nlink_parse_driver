#!/usr/bin/env python3
"""Read logical endpoints and per-observation provenance without inventing noise."""
import rospy
from liuo_uwb_msgs.msg import UwbObservationBatch

def receive(batch):
    for obs in batch.observations:
        rospy.loginfo('kind=%d edge=N%d->N%d reporter=N%d dataset=%d source_session=%d '
                      'source_seq=%d seq_valid=%s time_basis=%d source_stamp=%.9f '
                      'received=%.9f distance_m=%.3f rssi_valid=%s fp=%s rx=%s freshness=%s',
                      obs.kind, obs.node_a, obs.node_b, obs.reporter_id,
                      batch.driver_session_id, obs.source_session_id,
                      obs.source_frame_seq, obs.source_frame_seq_valid, obs.time_basis,
                      obs.source_frame_stamp.to_sec(), obs.received_stamp.to_sec(),
                      obs.distance_m, obs.rssi_valid, obs.fp_rssi, obs.rx_rssi,
                      obs.pair_measurement_freshness_verified)

if __name__ == '__main__':
    rospy.init_node('uwb_observation_reader')
    subscriber = rospy.Subscriber('/liuo/uwb/observations', UwbObservationBatch,
                                  receive, queue_size=100, tcp_nodelay=True)
    rospy.spin()
