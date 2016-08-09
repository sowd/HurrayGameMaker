#include <float.h>

void setup() {
  // Optional: Use Bean.setAccelerationRange() to set the sensitivity 
  // to something other than the default of ±2g.
  Serial.begin(9600) ;
  Bean.setLed(0, 0, 0);
}

const int LOG_COMPARE_LEN = 7 ;
const int LOG_AV_COUNT = 2 ;
const double LOOP_THR = 4000 * LOG_COMPARE_LEN ;
const int MINIMUM_OFFSET = 4 ;



const int LOGLEN = LOG_COMPARE_LEN*2 ;

double raw_accel[3] ;
int raw_accel_count = LOG_AV_COUNT ;


double pastlog[LOGLEN][3] ;
int logpos = 0 ;

double prevaccel[3] ;


// 0: stopped 1: finding loop 2: counting
int mode = 0 ;

int loopInterv = -1 ;
int loopInterv_countdown ;

void loop() {
  if( Serial.available()>0 ){
    char inChar = Serial.read();
    if( inChar == 's' ){
      mode = 1 ;
      for( int i=0;i<LOGLEN;++i )
        pastlog[i][0] = pastlog[i][1] = pastlog[i][2] = 0 ;
      logpos = 0 ;

      raw_accel[0] = raw_accel[1] = raw_accel[2] = 0 ;
      raw_accel_count = LOG_AV_COUNT ;

      Bean.setLed(255,0, 0);
    }
    if( inChar == 'e' ){
      Bean.setLed(0, 0, 0);
      mode = 0 ;
    }
  }
  if( mode == 0 ){
    Bean.sleep(100);
    return ;
  }

  AccelerationReading accel = Bean.getAcceleration();

  raw_accel[0] += accel.xAxis ;
  raw_accel[1] += accel.yAxis ;
  raw_accel[2] += accel.zAxis ;

  if( --raw_accel_count != 0 ) return ;
  raw_accel_count = LOG_AV_COUNT ;
  raw_accel[0] /= LOG_AV_COUNT ;
  raw_accel[1] /= LOG_AV_COUNT ;
  raw_accel[2] /= LOG_AV_COUNT ;

  pastlog[logpos][0] = raw_accel[0] - prevaccel[0] ;
  pastlog[logpos][1] = raw_accel[1] - prevaccel[1] ;
  pastlog[logpos][2] = raw_accel[2] - prevaccel[2] ;

  prevaccel[0] = raw_accel[0] ;
  prevaccel[1] = raw_accel[1] ;
  prevaccel[2] = raw_accel[2] ;

  raw_accel[0] = raw_accel[1] = raw_accel[2] = 0 ;

  boolean bIncreasing = false ;
  double prevCorr = DBL_MAX ;
  int i ;
  for( i=MINIMUM_OFFSET;i<LOGLEN-LOG_COMPARE_LEN;++i ){
    double corr = 0 ;
    for( int j=0;j<LOG_COMPARE_LEN;++j ){
      int s1 = (logpos  +j)%LOGLEN ;
      int s2 = (logpos+i+j)%LOGLEN ;
      corr += pastlog[s1][0] * pastlog[s2][0] + pastlog[s1][1] * pastlog[s2][1] + pastlog[s1][2] * pastlog[s2][2] ;
    }

    if( !bIncreasing && prevCorr < corr ) bIncreasing = true ;
    else if( bIncreasing && prevCorr > corr ){
      //prevCorr = corr ;
      break ;
    }
    prevCorr = corr ;
  }
  double peak = prevCorr ;

  logpos = (logpos+(LOGLEN-1)) % LOGLEN ;

  if( i == LOGLEN || peak < LOOP_THR ){
    loopInterv = -1 ; // Loop reset
    return ;
  }

  // Passed.
  if( loopInterv == -1 ){
    loopInterv = i ;
    loopInterv_countdown = i ;
    //Serial.print( "H" ) ;
  } else if(--loopInterv_countdown == 0 ){
    Serial.print( "H" ) ;
    //Serial.print( loopInterv ) ;
    loopInterv_countdown = loopInterv ;
  }
  //Bean.sleep(1000/15);
}
