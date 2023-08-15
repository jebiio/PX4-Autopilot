uint64 timestamp	# time since system start (microseconds)

int32 	msg_type      # AIS 메세지 유형 AIS message Type 링크 참고
int32 	repeat        # 반복 횟수
int32 	mmsi          # MMSI(Maritime Mobile Service Identity)
int32 	reserved_1    # 예약된 필드1
float32 speed       # 선박 속도 knots
uint8 	accuracy       # 위치 정보 정확도
float64 lon         # 경도
float64 lat         # 위도
float32 course      # COG
int32 	heading       # 헤딩
int32 	second        # 시간(초)
int32 	reserved_2    # 예약된 필드2
uint8 	cs             # 수평 추적 상태
uint8	 display        # AIS 표시기(Display)의 상태
uint8 	dsc            # DSC(Digital Selective Calling) 기능의 상태
uint8	 band           # 선박이 주파수 밴드를 선택하는 능력
uint8 	msg22          # 선박이 무선 전화 메시지를 수신할 수 있는 능력
uint8 	assigned       # 무선 주파수 채널이 선박에 할당되었는지 여부
uint8 	raim           # RAIM(Receiver Autonomous Integrity Monitoring) 기능의 상태
int32 	radio         #  선박의 무선 전화 번호


# float32 psi_d
# float32 spd_d
# int32 	msg_type      # AIS 메세지 유형 AIS message Type 링크 참고
# int32 	repeat        # 반복 횟수
# int32 	mmsi          # MMSI(Maritime Mobile Service Identity)
# int32 	reserved_1    # 예약된 필드1
# float32 speed       # 선박 속도 knots
# bool 	accuracy       # 위치 정보 정확도
# float64 lon         # 경도
# float64 lat         # 위도
# float32 course      # COG
# int32 	heading       # 헤딩
# int32 	second        # 시간(초)
# int32 	reserved_2    # 예약된 필드2
# bool 	cs             # 수평 추적 상태
# bool	 display        # AIS 표시기(Display)의 상태
# bool 	dsc            # DSC(Digital Selective Calling) 기능의 상태
# bool	 band           # 선박이 주파수 밴드를 선택하는 능력
# bool 	msg22          # 선박이 무선 전화 메시지를 수신할 수 있는 능력
# bool 	assigned       # 무선 주파수 채널이 선박에 할당되었는지 여부
# bool 	raim           # RAIM(Receiver Autonomous Integrity Monitoring) 기능의 상태
# int32 	radio         #  선박의 무선 전화 번호
