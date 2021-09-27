ntpd\_driver
============

This ROS node listen `sensor_msgs/TimeReference` and send it to ntpd via SHM (like gpsd).

Parameter `~/shm_unit` define SHM unit (in ntp.conf) (int, default: 2).
Parameter `~/time_ref_topic` define the topic to subscribe to (string, default: `"~/time_ref"`).
Parameter `~/fixup_date` enable/disable date fixup (bool, default: false)


System configuration
--------------------

### ntpd configuration

Add this to `/etc/ntp.conf`:

    ### GPS SHM driver
    server 127.127.28.2 minpoll 4 maxpoll 4
    fudge 127.127.28.2 time1 0.5 stratum 12 refid ROS

And then restart ntp service by running:
```
sudo systemctl restart ntp/systemd-timesyncd.service
```


Run example:
```
    rosrun ntpd_driver shm_driver _shm_unit:=2 _time_ref_topic:=/time_reference
```

To check the timesync status: 

```
ntpstat
```

To check the offset between system time and GPS time

```
ntpq -p
```
