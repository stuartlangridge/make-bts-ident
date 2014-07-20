import sys, random, os, time, datetime
from PIL import Image, ImageDraw

from Box2D import *

OUTPUT_SIZE = (800, 400)
WORLD_SIZE = (40, 20)


def simulate_world(step_function, optional_params=None):
    world = b2World()
    ground = world.CreateStaticBody(
                position=(0,0),
                shapes=[
                    b2EdgeShape(vertices=[(-(WORLD_SIZE[0]/2),0), ((WORLD_SIZE[0]/2),0)]),
                    b2EdgeShape(vertices=[((WORLD_SIZE[0]/2),0), ((WORLD_SIZE[0]/2),(WORLD_SIZE[1]))]),
                    b2EdgeShape(vertices=[(-(WORLD_SIZE[0]/2),0), (-(WORLD_SIZE[0]/2),(WORLD_SIZE[1]))]),
                ]
            )
    balls = []
    for i in range(4):
        if optional_params:
            posx = optional_params[i][0]
            rest = optional_params[i][1]
            velx = optional_params[i][2]
            vely = optional_params[i][3]
        else:
            while 1:
                posx = random.randint(-19, 19)
                rest = random.choice([0.3,0.4,0.5])
                velx = random.randint(-15, 15)
                vely = random.randint(-15, -5)
                if velx != 0 and vely != 0: break
        balls.append(world.CreateDynamicBody(
                position=(posx, 20),
                fixtures=b2FixtureDef(shape=b2CircleShape(radius=2), density=1.0, restitution=rest),
                linearVelocity=(velx, vely)
            ))
    params = [(
            int(x.position.x), x.fixtures[0].restitution,
            int(x.linearVelocity.x), int(x.linearVelocity.y))
        for x in balls]

    TARGET_FPS=60
    TIMESTEP=1.0/TARGET_FPS
    VEL_ITERS, POS_ITERS=8,3
    data = {"framecount": 0}
    while 1:
        world.Step(TIMESTEP, VEL_ITERS, POS_ITERS)
        world.ClearForces()
        stop = step_function(data, balls)
        if stop:
            break
    return balls, params

def tryworld():
    balls, params = simulate_world(count_function)
    success = check_function(balls)
    if success:
        return True, params
    return False, False

def count_function(data, balls):
    data["framecount"] += 1
    if data["framecount"] > 4.5 * 60:
        return True
    return False

def check_function(balls):
    # confirm the balls are all on the ground
    for vertdiff in [x.position.y - balls[0].position.y for x in balls]:
        if abs(vertdiff) > 0.1:
            return False
    # confirm they're all next to one another
    leftmost = min([x.position.x for x in balls])
    xdeltas = sorted([x.position.x - leftmost for x in balls])
    xdeltadiffs = [xdeltas[1] - xdeltas[0], xdeltas[2] - xdeltas[1], xdeltas[3] - xdeltas[2]]
    DIFF = 4.2
    for xd in xdeltadiffs:
        if xd > DIFF:
            return False
        if xd < DIFF - 0.2:
            return False
    # confirm they're not right at the edge
    if min([x.position.x for x in balls]) < (-(WORLD_SIZE[0]/2) + 4): return False
    if max([x.position.x for x in balls]) > ((WORLD_SIZE[0]/2) - 4): return False
    return True

def get_ball_position(ball):
    scalex = float(OUTPUT_SIZE[0]) / WORLD_SIZE[0]
    scaley = float(OUTPUT_SIZE[1]) / WORLD_SIZE[1]
    ballradius = 1.9

    ballboxtlx = ((ball.position.x - ballradius) * scalex) + (OUTPUT_SIZE[0]/2)
    ballboxtly = OUTPUT_SIZE[1] - ((ball.position.y - ballradius) * scaley)
    ballboxbrx = ((ball.position.x + ballradius) * scalex) + (OUTPUT_SIZE[0]/2)
    ballboxbry = OUTPUT_SIZE[1] - ((ball.position.y + ballradius) * scaley)
    tx = min(ballboxtlx, ballboxbrx)
    ty = min(ballboxtly, ballboxbry)
    bx = max(ballboxtlx, ballboxbrx)
    by = max(ballboxtly, ballboxbry)
    return (tx, ty, bx, by)

def make_png(data, balls):
    data["framecount"] += 1
    lastframe = False
    if data["framecount"] > 4.5 * 60:
        lastframe = True

    im = Image.new("RGB", OUTPUT_SIZE)
    draw = ImageDraw.Draw(im)
    if lastframe:
        deltax = get_ball_position(balls[1])[0] - get_ball_position(balls[0])[0]
    ballcount = 0
    for ball in balls:
        tx, ty, bx, by = get_ball_position(ball)
        if lastframe:
            # make all balls the same distance apart
            base = get_ball_position(balls[0])
            tx = base[0] + (deltax * ballcount)
            bx = base[2] + (deltax * ballcount)
        draw.ellipse([tx, ty, bx, by], fill=128)
    del draw
    im.save("seq/out-%03d.png" % data["framecount"], "PNG")

    if lastframe:
        return True
    return False

def make_video(params):
    #print "PYTHONPATH=pypybox2d-2.1-r331:pypybox2d-2.1-r331/testbed python balls.py ",
    #print " ".join(["%s %.1f %s %s" % x for x in params])
    #print params
    #print "make a video with params", params
    os.system("rm seq/out-*.png")
    endballs, endparams = simulate_world(make_png, params)
    # encode to video
    ofile = ["vidballs",
        datetime.datetime.strftime(datetime.datetime.now(), "%Y%m%d%H%M%S")]
    for ballp in params:
        for item in ballp:
            ofile.append(str(item))
    ofile = "_".join(ofile)
    os.system('''gst-launch-1.0 webmmux name=mux ! filesink location="''' + ofile + '''.webm"    file:///`pwd`/damian_turnbull_idents_piano__damian_turnbull__preview.mp3 ! decodebin ! audioconvert ! vorbisenc ! mux.     multifilesrc location="seq/out-%03d.png" index=1 caps="image/png,framerate=\(fraction\)70/1" ! pngdec ! videoconvert ! videoscale ! videorate ! vp8enc threads=4 ! mux.''')
    os.system("rm seq/out-*.png")
    print "Made video", ofile
    print "Kill the program now..."
    time.sleep(2)


if __name__ == "__main__":
    while 1:
        while 1:
            success, params = tryworld()
            if success: break
        make_video(params)
