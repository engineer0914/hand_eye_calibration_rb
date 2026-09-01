# 6d X,Y,Z,RX,RY,RZ를 반환 하는 코드
# 사용 예시로 포지션 변수 = ND.ARRAY([반환된 6D])

import rbpodo as rb

ROBOT_IP = "192.168.0.22"


def _main():
    try:
        robot = rb.Cobot(ROBOT_IP)
        rc = rb.ResponseCollector()

        response, tcp = robot.get_tcp_info(rc)

        print(response)
        print("[" + ", ".join(f"{value:.2f}" for value in tcp) + "]")

    except Exception as e:
        print(e)


if __name__ == "__main__":
    _main()



