#ifndef STATE_H
#define  STATE_H 
enum State {
  START_POINT,
  SENTIER_ANCIENT,// curved path 
  HEXAGONE,
  SENTIER_BIRSE, // interrupted line
  SENTIER_BIRSE_2,// GO_Forwad_ignore,
  REPOS_ANCIENT, // stop 5 s
  PASSAGE_CERCLE,// GO left 
  ESCALIER_DEFI,// ligne interempu
  CHEMIN_SINUSOIDALE,// inverted path
  TRIANGLE_LABYRINTHE,// ZIZAG PATH
  END_POINT,// have to stop 
// tests states 
  TEST_STOP_MOTOR,
  TEST,
  TEST_Motor,
  TEST_BLUETOOTH,
  TEST_FOLLOW_LINE,
  END_STATE,

};

#endif



