val rawLines=scala.io.Source.fromFile("path-clean.yaml").getLines.map( _.trim ).toSeq

// Parsing to points
case class Position( val x:Double, val y:Double, val z:Double )
case class Orientation( val x:Double, val y:Double, val z:Double, val w:Double )
case class Point( val pos:Position, val orientation: Orientation )

val yamlVal = (x:String) => x.split(":")(1).toDouble

val rawGroups = rawLines.foldRight(List(List[String]())
    )( (i,l) => { if (i == "pose:") List[String]()::l
            else (i +: l.head) :: l.tail }
    ).tail

val points = rawGroups.map( rg => Point( Position( yamlVal(rg(1)), yamlVal(rg(2)), yamlVal(rg(3))),
                             Orientation(yamlVal(rg(5)), yamlVal(rg(6)), yamlVal(rg(7)), yamlVal(rg(8)))))
// Print path
println
println
print("TSV of the positions")
println
print("idx\tx\ty\tz" )
println
points.zipWithIndex.foreach( pi => print( Array(pi._2.toString, pi._1.pos.x, pi._1.pos.y, pi._1.pos.z).mkString("\t")+"\n") )
