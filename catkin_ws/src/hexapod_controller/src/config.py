#!coding:utf-8
# 183/2
kLegMiddleX = 91.5
# 117/2
kLegOtherX = 58.5
# 237/2
kLegOtherY = 118.5

kLegJ1ToJ2 = 45
kLegJ2ToJ3 = 75
kLegJ3ToTip = 145

SIN30 = 0.5
COS30 = 0.866
SIN45 = 0.7071
COS45 = 0.7071
SIN15 = 0.2588
COS15 = 0.9659

STANDBY_Z = (kLegJ3ToTip*COS15-kLegJ2ToJ3*SIN30)
MIDDLE_X = (kLegMiddleX +
            kLegJ1ToJ2+(kLegJ2ToJ3*COS30)+kLegJ3ToTip*SIN15)
OTHER_X = (kLegOtherX + (kLegJ1ToJ2 +
                         (kLegJ2ToJ3*COS30)+kLegJ3ToTip*SIN15)*COS45)
OTHER_Y = (kLegOtherY + (kLegJ1ToJ2 +
                         (kLegJ2ToJ3*COS30)+kLegJ3ToTip*SIN15)*SIN45)

# 0 30 15
defaultPosition = (
    (OTHER_X, OTHER_Y, -STANDBY_Z),
    (MIDDLE_X, 0, -STANDBY_Z),
    (OTHER_X, -OTHER_Y, -STANDBY_Z),
    (-OTHER_X, -OTHER_Y, -STANDBY_Z),
    (-MIDDLE_X, 0, -STANDBY_Z),
    (-OTHER_X, OTHER_Y, -STANDBY_Z),
)

# 每只脚的相对坐标原点
mountPosition = (
    (kLegOtherX, kLegOtherY, 0),
    (kLegMiddleX, 0, 0),
    (kLegOtherX, -kLegOtherY, 0),
    (-kLegOtherX, -kLegOtherY, 0),
    (-kLegMiddleX, 0, 0),
    (-kLegOtherX, kLegOtherY, 0),
)

defaultAngle = (
    -45, 0, 45, 135, 180, 225
)

angleLimitation = (
    (-45, 45),
    (-45, 75),
    (-60, 60),
)

if __name__ == '__main__':
    print('defaultPosition')
    for i in defaultPosition:
        print(i)
    print('mountPosition')
    for i in mountPosition:
        print(i)
