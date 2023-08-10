uint64 timestamp	# time since system start (microseconds)

uint8 nav_mode          # 항법모드 (single/RTK/INS)
float32 nav_roll        # degree roll
float32 nav_pitch       # degree pitch
float32 nav_yaw         # degree yaw
float32 nav_yaw_rate    # [degree/s] yaw_rate
float32 nav_cog         # degree COG
float32 nav_sog         # knots degree SOG
float32 nav_uspd        # konts longitudinal speed
float32 nav_vspd        # knots traversal speed
float32 nav_wspd        # knots vertical speed
float64 nav_longitude   # degree 경도
float64 nav_latitude    # degree 위도
float32 nav_heave       # m Heave
float32 nav_gpstime     #  hhmmss.s GPS시간
float32 wea_airtem      # degree 공기 온도
float32 wea_wattem      # degree 해수 온도
float32 wea_press       # bar 기압
float32 wea_relhum      # % 상대습도
float32 wea_dewpt       # degree 이슬점
float32 wea_windirt     # degree 풍향(절대)
float32 wea_winspdt     # knots 풍속(절대)
float32 wea_windirr     # degree 풍향(상대)
float32 wea_watspdr     # knots 풍속(상대)
float32 wea_watdir      # degree 유향
float32 wea_watspd      # knots 유속
float32 wea_visibiran   # m 가시거리
