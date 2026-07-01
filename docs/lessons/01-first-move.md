# Lesson 1 — Your First Move

**What you will learn:** How to connect to the robot, drive it forward, and stop it.

---

## The code

Copy this into a new file called `lesson1.py` and run it:

```python
import bugbot; bugbot.go()   # always the first line

forward(50)    # drive forward at 50% speed
wait(2)        # wait 2 seconds
stop()         # stop all motors
```

Run it:

```
python lesson1.py
```

The robot should drive forward for two seconds and then stop.

---

## What's happening

```python
import bugbot; bugbot.go()
```

This loads the BugBot library and connects to the robot. It also makes all the
robot functions — `forward`, `stop`, `distance`, etc. — available in your script.
**This line must always be first.**

```python
forward(50)
```

This starts the robot driving forward. The number is the speed — `0` is stopped,
`100` is full power, `50` is half. The robot keeps driving until you tell it to stop.

```python
wait(2)
```

This pauses your program for 2 seconds. While your program is paused, the robot
keeps doing whatever it was doing — in this case, driving forward.

```python
stop()
```

This stops all four motors immediately.

---

## Challenges

1. **Change the speed.** Try `forward(20)` and `forward(80)`. How does it feel different?

2. **Go backward.** Replace `forward(50)` with `backward(50)`. What happens?

3. **Drive in a square.** Drive forward, stop, turn, stop, and repeat four times.
   Hint: `turn(90)` turns the robot 90° clockwise.

4. **Long drive.** Make the robot drive forward for 10 seconds by changing the number inside `wait()`.

5. **Add a beep.** Use `beep(440, 200)` to make the robot beep when it stops.

---

## Bonus: Robot class style

If you already know Python well, you can also use the `Robot` class directly.
This gives you more control but requires a bit more code:

```python
from bugbot import Robot

with Robot(0) as bot:
    bot.forward(50)
    import time; time.sleep(2)
    bot.stop()
# disconnects automatically here
```

Both styles talk to the same robot — use whichever feels clearer to you.
