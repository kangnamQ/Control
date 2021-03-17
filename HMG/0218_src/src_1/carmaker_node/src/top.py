

## 신호등 o
## 표지판
## 비상정지


if  : # 양쪽 라인이 -1.5<x<1.5일 경우
	##3 line 모드
	mode = "line"
	##돌기

if : # 방향지시등이 0이 아닐경우
	mode = "turn"

if : # 로컬 신호가 들어왔을 경우
	mode = "local"

if mode is not "line" or "turn" or "local":
	mode = "waypoint"



if : # 라이다 검출이되면
	ACC == "on"
else: # 라이다 검출 안되면
	ACC == "off"

if : # 핸들
	pure = "off"
else :
	pure = "on"


if mode == "line":
	# Use : ACC, lkas, PID
	# not use : pure
	pure = off
	line_streeing = on
	line => # 사용
	if ACC == "on":
		acc
	elif ACC == "off":
		pid

elif mode == "turn":
	# Use : pure, PID   but gain 값 변화
	# not use : ACC, lkas	라이다는 사용안함

	ACC == "off":
		pid 주

elif mode == "local":
	# Use : local_point, PID, acc  모두 local로 다시 짜서 주행
	# not use : lkas, global_point
	pure = "on"
	line_streeing = "off"
	if ACC == "on":
		acc
	elif ACC == "off":
		pid

elif mode == "waypoint":
	# Use : pure, PID, acc  모두 local로 다시 짜서 주행
	# not use : lkas

