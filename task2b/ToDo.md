Aim
Fill in detect_signs() and detect_obstacle() of the Task 2B boilerplate so that for a given image they report every road sign visible and whether the obstacle is present and how large each one appears.

Files and Setup
Both files come from the theme’s public repository - see Getting the Code on the overview page.

File	What it is
task2b/sign_obstacle_detection.py	the boilerplate you fill in
task2b/public/	the public set of images to develop against
task2b/reference_images/	Parking/Hospital reference crops, if your approach wants them
task2b/
├── sign_obstacle_detection.py     <- the boilerplate, with your detect_signs() and detect_obstacle()
├── reference_images/
│   ├── P.jpg
│   └── Hospital.png
└── public/
    ├── img_01.jpg
    ├── img_02.jpg
    └── …

These are the public images: every team gets the same ones, for developing on your own machine. Your submission is also run against a private set you never see, so a detector tuned to these particular pictures will not carry over. Exactly how the marks are split is on the Submission page.

reference_images/ holds clean crops of the Parking and Hospital letters. It is useful if your approach to telling those two signs apart involves matching against a reference, rather than something purely rule-based. Keep this folder in the same directory as your script, both while developing and when you run the evaluation tool. You must NOT, include it in your submission zip (see Submission) beacause the private grader keeps its own copy of the exact same folder and places it next to your script before running it, so a path resolved the same way will still work there.

This file has no main() of its own - it only defines the two functions below. Use the evaluation tool described on the Submission page to run your code against the public images; see Running It further down.

The Functions You Implement
detect_signs()

def detect_signs(image):
    ...
    return detections

Return	Type	Meaning
detections	list of dict	Zero, one, or many entries - one per sign visible in image. An image can hold more than one sign at once.
detections[i]["label"]	str	One of SIGN_LABELS: "School", "Speed40", "Parking", "Hospital"
detections[i]["area"]	int	That sign’s area in pixels
An image with no sign in it should return [], not a guess. Returning a sign that is not really there costs you exactly as much as missing one that is - see Submission for how detections are matched to the ground truth.

detect_obstacle()

def detect_obstacle(image):
    ...
    return {"present": present, "area": area}

Return	Type	Meaning
present	bool	Whether the obstacle is visible in image
area	int	Its area in pixels; 0 when present is False
Adding Helper Functions
You are not expected to fit everything into these two functions. Add as many helper functions and global variables as you like - put them below detect_obstacle() but above the END OF YOUR IMPLEMENTATION line, and call them from either function:

def detect_signs(image):
    ...
    red_mask = isolate_colour(image, RED_RANGES)      # your helper, defined below
    ...
    return detections


# ---- helpers, still inside the implementation block ----

def isolate_colour(image, ranges):
    ...
    return mask