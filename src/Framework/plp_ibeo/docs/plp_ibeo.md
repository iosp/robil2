# WP Navigation PLP

1. 2015-03-01 Initial Version

## About
This PLP **detects** when an IBEO unit cannot provide useful sensing data.

## Description (Daniel)
<div style="direction:rtl; white-space:pre-line">

אתה יכול להירשם ל topic של 1\\SENSORS\IBEO

אתה תראה את כל הנתונים כפי שמפורט ומוסבר ב ICD שמצורף תחת סעיף robil_msgs/MultiLaserScan

אתה יכול לבצע בדיקה על ההיבטים הבאים :

1.      time_increment לוודא שלא חורג מ sec0.08 (12.5Hz) פלוס מינוס sec0.001  ושבאמת תואם לקצב הגעת ההודעות.
2.      ranges_t1[] , ranges_t2[], ranges_t3[], ranges_t4[]  - אלה המערכים של הסריקות של ארבעת הקרניים, אתה יכול לבדוק עבורם
a.       אחוז הנק' עם טווח קטן מ- 1מ'  -   לבדוק שהוא לא גדול מ-  10%  -- מצבים בהם החישן לא תקין או שיש משהו שמסתיר (כמו הכף של ה BobCat)
b.      אחוז הנק' עם טווח קטן מ- 3מ' - לבדוק שהוא לא גדול מ- מ 30%  -- מצבים שיש גוף גדול ומאוד קרוב ל BobCat
c.       אחוז הנק' עם טווח גדול מ- 50מ' - לבדוק שהוא לא גדול מ- מ 80%    --  מצבים שהחיישן לא תקין או מסתכל לשמיים


בוא נתחיל מזה, אחרי שנצליח נוסיף בהמשך עוד פרמטרים לבדיקה.

את האחוזים בשלב זה קבעתי שרירותית, לכן כדי שילקחו מקובץ קונפיגורציה.
</div>


## Values
### Parameters
* `scan` &larr; `/SENSORS/IBEO/1` Scan data from the IBEO node.

### Constants
* `TIME_INCREMENT` Time interval between IBEO scans (0.08sec)
* `TIME_INCREMENT_TOLERANCE` (0.001sec)
* `FAIL_OR_COVER_THRESHOLD` Percent of points below 1m. Above this, indicates sensor failure or something covering the IBEO, e.g. Bobcat's arm (10%)
* `OBSTACLE_THRESHOLD` Percent of points below 3m. Above this - there's an obstacle close to the Bobcat (30%)
* `SKY_THRESHOLD` Percent of points further than 50m. Above this, IBEO is probably pointing to the sky (80%)

### Variables
_Calculated based on parameters and constants._

* `last_scan_time` last time scan happened.
* `fail_or_cover_pcnt[i]` how many reads on scan `i` are below 1m.
* `obstacle_pcnt[i]` how many reads on scan `i` are below 3m.
* `sky_pcnt[i]` how many reads on scan `i` are above 50m.


## Application Context
### Resources
* IBEO (shared)

### Preconditions
* None

### Concurrency Conditions
_Later, we will replace this with "health" PLP for the entire system_

* `IBEO_OK`

### Concurrent Modules
* None

## Side Effects
* None

## Detection Goals
* For each i in [0..3]:
  * `fail_or_cover_pcnt[i] > FAIL_OR_COVER_THRESHOLD`
  * `obstacle_pcnt[i] > OBSTACLE_THRESHOLD`
  * `sky_pcnt[i] > SKY_THRESHOLD`
  * `current_time in Range(last_scan_time+TIME_INCREMENT-TIME_INCREMENT_TOLERANCE,
                          last_scan_time+TIME_INCREMENT+TIME_INCREMENT_TOLERANCE)`

## Detection Probability Given Condition
0.99

## Robil Integration

### Trigger
* System startup

### Abort
* None

### Output
* Predictions and advance measure warnings sent to the `plp/messages` ROS topic.
