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
    return balls, params, data

def tryworld():
    balls, params, data = simulate_world(count_function)
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

def get_ball_position_from_position(position):
    scalex = float(OUTPUT_SIZE[0]) / WORLD_SIZE[0]
    scaley = float(OUTPUT_SIZE[1]) / WORLD_SIZE[1]
    ballradius = 1.9

    ballboxtlx = ((position[0] - ballradius) * scalex) + (OUTPUT_SIZE[0]/2)
    ballboxtly = OUTPUT_SIZE[1] - ((position[1] - ballradius) * scaley)
    ballboxbrx = ((position[0] + ballradius) * scalex) + (OUTPUT_SIZE[0]/2)
    ballboxbry = OUTPUT_SIZE[1] - ((position[1] + ballradius) * scaley)
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

    im = Image.new("RGB", OUTPUT_SIZE, color=(200, 200, 255))
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
        draw_a_sphere([tx, ty, bx, by], im, ballcount, data["framecount"])
        #draw.ellipse([tx, ty, bx, by], fill=128)
        ballcount += 1
    del draw
    im.save("seq/out-%03d.png" % data["framecount"], "PNG")

    if lastframe:
        return True
    return False

def convert_point(x, y, step, steptotal, source_rect, destination_rect):
    x_as_fraction = (x - source_rect[0]) / source_rect[2]
    y_as_fraction = (y - source_rect[1]) / source_rect[3]
    new_ultimate_x = (x_as_fraction * (destination_rect[2] - destination_rect[0])) + destination_rect[0]
    new_ultimate_y = (y_as_fraction * (destination_rect[3] - destination_rect[1])) + destination_rect[1]
    dx = new_ultimate_x - x
    dy = new_ultimate_y - y
    new_x = x + (dx * float(step) / steptotal)
    new_y = y + (dy * float(step) / steptotal)
    return (new_x, new_y)

sphere = Image.open("green-sphere.png")
def draw_a_sphere(box, im, ballindex, frame):
    tx, ty, bx, by = box
    sphsize = sphere.resize((int(bx-tx), int(by-ty)), Image.ANTIALIAS)
    im.paste(sphsize, box=(int(tx), int(ty)), mask=sphsize)

def make_video(params):
    #print "PYTHONPATH=pypybox2d-2.1-r331:pypybox2d-2.1-r331/testbed python balls.py ",
    #print " ".join(["%s %.1f %s %s" % x for x in params])
    #print params
    #print "make a video with params", params
    print "Making a video"
    os.system("rm seq/out-*.png")
    balls, endparams, data = simulate_world(make_png, params)
    positions = [(x.position.x, x.position.y) for x in balls]
    # create all the remaining frames
    STAND_STILL_FRAMES = 60
    print "  still frames..."
    for i in range(STAND_STILL_FRAMES): # static frames
        im = Image.new("RGBA", OUTPUT_SIZE, color=(200, 200, 255))
        draw = ImageDraw.Draw(im)
        deltax = get_ball_position_from_position(positions[1])[0] - get_ball_position_from_position(positions[0])[0]
        ballcount = 0
        for ballindex in range(len(balls)):
            tx, ty, bx, by = get_ball_position_from_position(positions[ballindex])
            col = (0, 255, 0)
            if i > STAND_STILL_FRAMES - 10:
                col = (0, 0, 255)
            draw_a_sphere([tx, ty, bx, by], im, ballindex, data["framecount"] + i)
            ballcount += 1
        del draw
        im.save("seq/out-%03d.png" % (int(data["framecount"]) + i), "PNG")
    SPIN_FRAMES = 120
    print "  spin frames..."
    zoom_down_to_fraction_of_total = 3
    zoom_down_to_tl = (
        OUTPUT_SIZE[0] * ((zoom_down_to_fraction_of_total - 1.5) / zoom_down_to_fraction_of_total), 
        OUTPUT_SIZE[1] * ((zoom_down_to_fraction_of_total - 1.5) / zoom_down_to_fraction_of_total)
    )
    zoom_down_to = ( zoom_down_to_tl[0], zoom_down_to_tl[1], 
        (OUTPUT_SIZE[0] / zoom_down_to_fraction_of_total) + zoom_down_to_tl[0], 
        (OUTPUT_SIZE[1] / zoom_down_to_fraction_of_total) + zoom_down_to_tl[1]
    )
    for i in range(SPIN_FRAMES): # static frames
        print "spin frame", i
        im = Image.new("RGB", OUTPUT_SIZE, color=(200, 200, 255))
        draw = ImageDraw.Draw(im)

        # draw other circles
        sortpos = sorted(positions, cmp=lambda a,b: cmp(a[0], b[0]))
        circ_0 = get_ball_position_from_position(sortpos[0])
        circ_1 = get_ball_position_from_position(sortpos[1])
        converted_0_tl = convert_point(circ_0[0], circ_0[1], i+1, SPIN_FRAMES, (0, 0, OUTPUT_SIZE[0], OUTPUT_SIZE[1]), zoom_down_to)
        converted_1_tl = convert_point(circ_1[0], circ_1[1], i+1, SPIN_FRAMES, (0, 0, OUTPUT_SIZE[0], OUTPUT_SIZE[1]), zoom_down_to)
        converted_0_br = convert_point(circ_0[2], circ_0[3], i+1, SPIN_FRAMES, (0, 0, OUTPUT_SIZE[0], OUTPUT_SIZE[1]), zoom_down_to)
        converted_1_br = convert_point(circ_1[2], circ_1[3], i+1, SPIN_FRAMES, (0, 0, OUTPUT_SIZE[0], OUTPUT_SIZE[1]), zoom_down_to)
        circ_size = converted_0_br[0] - converted_0_tl[0]
        circ_plus_gap_size = converted_1_tl[0] - converted_0_tl[0]
        startx = converted_0_tl[0]
        while startx > 0: startx = startx - circ_plus_gap_size
        starty = converted_0_tl[1]
        while starty > 0: starty = starty - circ_plus_gap_size
        endx = startx
        while endx < OUTPUT_SIZE[0]: endx += circ_plus_gap_size
        endy = starty
        while endy < OUTPUT_SIZE[1]: endy += circ_plus_gap_size
        xpos = startx
        while 1:
            ypos = starty
            while 1:
                #draw.ellipse([xpos, ypos, xpos + circ_size, ypos + circ_size], fill=(200,200,200))
                draw_a_sphere([xpos, ypos, xpos + circ_size, ypos + circ_size], im, 5, 
                    data["framecount"] + STAND_STILL_FRAMES + i) # ballindex 5 for the "other" spheres
                ypos += circ_plus_gap_size
                if ypos >= endy: break
            xpos += circ_plus_gap_size
            if xpos >= endx: break


        deltax = get_ball_position_from_position(positions[1])[0] - get_ball_position_from_position(positions[0])[0]
        ballcount = 0
        for ballindex in range(len(balls)):
            tx, ty, bx, by = get_ball_position_from_position(positions[ballindex])
            col = (255, 255, 0)
            if i > SPIN_FRAMES - 10:
                col = (0, 255, 255)
            ttx, tty = convert_point(tx, ty, i+1, SPIN_FRAMES, (0, 0, OUTPUT_SIZE[0], OUTPUT_SIZE[1]), zoom_down_to)
            bbx, bby = convert_point(bx, by, i+1, SPIN_FRAMES, (0, 0, OUTPUT_SIZE[0], OUTPUT_SIZE[1]), zoom_down_to)
            #draw.ellipse([ttx, tty, bbx, bby], fill=col)
            draw_a_sphere([ttx, tty, bbx, bby], im, ballindex, data["framecount"] + STAND_STILL_FRAMES + i)
            ballcount += 1

        del draw
        im.save("seq/out-%03d.png" % (int(data["framecount"]) + i + STAND_STILL_FRAMES), "PNG")

    # encode to video
    ofile = ["vidballs",
        datetime.datetime.strftime(datetime.datetime.now(), "%Y%m%d%H%M%S")]
    for ballp in params:
        for item in ballp:
            ofile.append(str(item))
    ofile = "_".join(ofile)
    os.system('''gst-launch-1.0 webmmux name=mux ! filesink location="''' + ofile + '''.webm"    file:///`pwd`/damian_turnbull_idents_piano__damian_turnbull__hq.wav ! decodebin ! audioconvert ! vorbisenc ! mux.     multifilesrc location="seq/out-%03d.png" index=1 caps="image/png,framerate=\(fraction\)70/1" ! pngdec ! videoconvert ! videoscale ! videorate ! vp8enc threads=4 ! mux.''')
    print "Made video", ofile
    print "Kill the program now..."
    time.sleep(2)


if __name__ == "__main__":
    while 1:
        while 1:
            success, params = tryworld()
            if success: break
        make_video(params)
