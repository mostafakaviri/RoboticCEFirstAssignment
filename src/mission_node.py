#!/usr/bin/python3

from first.srv import GetNextDestination, GetNextDestinationResponse, GetNextDestinationRequest
import rospy, random


def GetRandomDestination(req : GetNextDestinationRequest):
    
    res = GetNextDestinationResponse()

    a = req.current_x +5
    b = req.current_x -5
    c = req.current_y +5
    d = req.current_y -5
    
    if a > 10 :
        res.next_x = random.randint(b, 10)
    if b < -10 :
        res.next_x = random.randint(-10, a)
    if c > 10 :
        res.next_y = random.randint(d, 10)
    if d < -10 :
        res.next_y = random.randint(-10, c)

    else:

        e = random.randint(a, 10)
        f = random.randint(-10, b)
        g = random.randint(c, 10)
        h = random.randint(-10, d)
        
        i = []
        j = []

        i.append(e)
        i.append(f)

        j.append(g)
        j.append(h)

        res.next_x = random.choice(i)
        res.next_y = random.choice(j)
        
    return res

rospy.init_node('mission_node', anonymous=True)

s = rospy.Service('/mammad', GetNextDestination, GetRandomDestination)


rospy.spin()
