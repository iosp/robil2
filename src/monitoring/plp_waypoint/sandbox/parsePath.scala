val rawLines=scala.io.Source.fromFile("path-clean.yaml").getLines.map( _.trim ).toSeq

// Parsing to points
var x=0.0
var y=0.0
var inPoint=false

case class Point( val x:Double, val y:Double )
val points = scala.collection.mutable.ListBuffer[Point]()

rawLines.foreach( line => {
  val parts = line.split(":")
  parts(0) match {
    case "pose" => inPoint=true
    case "x" => x = parts(1).toDouble
    case "y" => { if (inPoint) {
      y = parts(1).toDouble
      points += Point(x,y)
      inPoint = false
    }}
    case _ => null
}})

// Print path
points.zipWithIndex.foreach( p => print( Array(p._2, p._1.x, p._1.y).mkString("\t")+"\n") )
