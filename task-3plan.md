## Task 1C - Instructions

# Aim
Fill in the detect_lane() function of the Task 1C boilerplate so that, for every frame of the given videos, it reports where the centre of the lane is and which lane the vehicle is in.

## things to keep in mind

conda activate NV_<Team-ID>
python lane_detection.py public/clip_01.mp4 --show

the 20 video clips, already in the repository: task1c/public/

## Function

The Function You Implement

detect_lane() is the only function you write. It receives one frame and returns one dictionary:

def detect_lane(frame):
    ...
    return {"center_x": center_x, "lane": lane}

Which lane is which. The dashed white line divides the two lanes, so the side of it that the vehicle is on decides the answer:

The dashed centre line is…	lane
to the right of the vehicle	"left"
to the left of the vehicle	"right"
not identifiable in this frame	"unknown"

## Frame Resolution and Coordinates

The clips are 640 x 480, and center_x is an absolute pixel column in the frame as you received it. The evaluator compares it against a ground truth measured in those same pixels, so the number only means anything in them.

Report the answer in the frame’s own coordinates

Inside detect_lane() you may resize, crop or warp as much as you like - but convert the answer back before you return it.

    Found the centre in a 320 x 240 copy? It is half the value it should be. Scale it back up.
    Read it off a bird’s eye view? That is a warped coordinate, not a frame one. Map the point back through the inverse of your perspective transform (cv2.perspectiveTransform with the inverse matrix).
    Do not re-encode, resize or re-compress the clip files themselves. The evaluator checks every clip is 640 x 480 and refuses to grade a dataset that is not, because every correct answer would be shifted sideways.
## Adding Helper Functions

You are not expected to fit everything into one function. Add as many helper functions and global variables as you like - put them below detect_lane() but above the END OF YOUR IMPLEMENTATION line, and call them from detect_lane():

def detect_lane(frame):
    ...
    yellow_x, white_x = marking_positions(frame)      # your helper, defined below
    ...
    return {"center_x": center_x, "lane": lane}


#helpers, still inside the implementation block ----

def marking_positions(frame):
    ...
    return yellow_x, white_x

## Rules
Read this before you start writing code

    Write your code only inside the block marked ADD YOUR IMPLEMENTATION HERE. Everything outside it - validate_result(), draw_overlay(), process_video(), main() - must stay exactly as shipped, because the evaluation script depends on it.
    No display or printing inside detect_lane(). No cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print(). The function computes and returns, nothing else. All visualisation belongs in draw_overlay(), which process_video() already calls for you when you pass --show.
    Do not rename the file, the function, or the keys of the returned dictionary.
    Helper functions and global variables are welcome, but they must live inside the marked block and be reachable from detect_lane().

## Before You Submit

    Check the --show overlay through a turn, not only on the straights.
    Confirm the lane label stays stable when the dashed line is passing through a gap.
    Comment out or remove any debugging you added inside detect_lane().
