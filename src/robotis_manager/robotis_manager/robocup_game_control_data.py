#!/usr/bin/python

from construct import Array, Byte, Const, Int16sl, Int16ul, Struct

GAMECONTROLLER_DATA_PORT = 3838
GAMECONTROLLER_RETURN_PORT = 3939

GAMECONTROLLER_STRUCT_HEADER = b'RGme'
GAMECONTROLLER_STRUCT_VERSION = 8

MAX_NUM_PLAYERS = 11

# SPL
TEAM_BLUE = 0
TEAM_CYAN = 0
TEAM_RED = 1
TEAM_MAGENTA = 1
DROPBALL = 2
# TEAM_BLUE = 0  # blue, cyan
# TEAM_RED = 1  # red, magenta, pink
# TEAM_YELLOW = 2  # yellow
# TEAM_BLACK = 3  # black, dark gray
# TEAM_WHITE = 4  # white
# TEAM_GREEN = 5  # green
# TEAM_ORANGE = 6  # orange
# TEAM_PURPLE = 7  # purple, violet
# TEAM_BROWN = 8  # brown
# TEAM_GRAY = 9  # lighter gray

# COMPETITION_PHASE_ROUNDROBIN = 0
# COMPETITION_PHASE_PLAYOFF = 1

# COMPETITION_TYPE_NORMAL = 0
# COMPETITION_TYPE_CHALLENGE_SHIELD = 1
# COMPETITION_TYPE_7V7 = 2
# COMPETITION_TYPE_DYNAMIC_BALL_HANDLING = 3

# GAME_PHASE_NORMAL = 0
# GAME_PHASE_PENALTYSHOOT = 1
# GAME_PHASE_OVERTIME = 2
# GAME_PHASE_TIMEOUT = 3

STATE_INITIAL = 0
STATE_READY = 1
STATE_SET = 2
STATE_PLAYING = 3
STATE_FINISHED = 4

# SET_PLAY_NORMAL = 0
# SET_PLAY_GOAL_KICK = 1
# SET_PLAY_PUSHING_FREE_KICK = 2
# SET_PLAY_CORNER_KICK = 3
# SET_PLAY_KICK_IN = 4
# SET_PLAY_PENALTY_KICK = 5
STATE2_NORMAL = 0
STATE2_PENALTYSHOOT = 1
STATE2_OVERTIME = 2
STATE2_TIMEOUT = 3

PENALTY_NONE = 0
# SPL
PENALTY_SPL_ILLEGAL_BALL_CONTACT = 1  # ball holding / playing with hands
PENALTY_SPL_PLAYER_PUSHING = 2
PENALTY_SPL_ILLEGAL_MOTION_IN_SET = 3  # heard whistle too early?
PENALTY_SPL_INACTIVE_PLAYER = 4  # fallen, inactive
PENALTY_SPL_ILLEGAL_POSITION = 5
PENALTY_SPL_LEAVING_THE_FIELD = 6
PENALTY_SPL_REQUEST_FOR_PICKUP = 7
PENALTY_SPL_LOCAL_GAME_STUCK = 8
PENALTY_SPL_ILLEGAL_POSITION_IN_SET = 9
# HL_KID
PENALTY_HL_KID_BALL_MANIPULATION = 1
PENALTY_HL_KID_PHYSICAL_CONTACT = 2
PENALTY_HL_KID_ILLEGAL_ATTACK = 3
PENALTY_HL_KID_ILLEGAL_DEFENSE = 4
PENALTY_HL_KID_REQUEST_FOR_PICKUP = 5
PENALTY_HL_KID_REQUEST_FOR_SERVICE = 6
PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE = 7

PENALTY_SUBSTITUTE = 14
PENALTY_MANUAL = 15

RobotInfo = Struct(
    'penalty' / Byte,  # penalty state of the player
    'secsTillUnpenalised' / Byte  # estimate of time till unpenalised
)

TeamInfo = Struct(
    'teamNumber' / Byte,  # unique team number
    'teamColour' / Byte,  # colour of the team
    'score' / Byte,  # team's score
    'penaltyShot' / Byte,  # penalty shot counter
    'singleShots' / Int16ul,  # bits represent penalty shot success  # noqa: E501
    'messageBudget' / Int16ul,
    # number of team messages the team is allowed to send for the remainder of the game  # noqa: E501
    'players' / Array(MAX_NUM_PLAYERS, RobotInfo)  # the team's players
)

RoboCupGameControlData = Struct(
    'header' / Const(GAMECONTROLLER_STRUCT_HEADER),  # header to identify the structure  # noqa: E501
    'version' / Const(GAMECONTROLLER_STRUCT_VERSION, Byte),  # version of the data structure  # noqa: E501
    'packetNumber' / Byte,  # number incremented with each packet sent (with wraparound)  # noqa: E501
    'playersPerTeam' / Byte,  # the number of players on a team
    'state' / Byte,  # state of the game (STATE_READY, STATE_PLAYING, etc)  # noqa: E501
    'firstHalf' / Byte,  # 1 = game in first half, 0 otherwise
    'kickOffTeam' / Byte,  # the team number of the next team to kick off, free kick etc
    'dropInTeam' / Byte,  # team that caused last drop in
    'dropInTime' / Int16sl,  # number of seconds passed since the last drop in.  -1 before first dropin
    'secsRemaining' / Int16sl,  # estimate of number of seconds remaining in the half
    'secondaryTime' / Int16sl,
    # number of seconds shown as secondary time (remaining ready, until free ball, etc)  # noqa: E501
    'teams' / Array(2, TeamInfo)
    # 'competitionPhase' / Byte,  # phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)  # noqa: E501
    # 'competitionType' / Byte,   # type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_CHALLENGE_SHIELD, etc)  # noqa: E501
    # 'gamePhase' / Byte,         # phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)  # noqa: E501
    # 'setPlay' / Byte,           # active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)  # noqa: E501
    # 'secondaryState' / Byte,     #Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc) # noqa: E501
)
