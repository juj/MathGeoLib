<?php
date_default_timezone_set("Europe/Helsinki");

include_once("geshi.php");

$sqlConnectionLink = 0;
$sqlAddress = "localhost"; // Configure this!
$sqlUserName = "username"; // Configure this! 
$sqlPassword = "password";// Configure this!
$sqlDatabaseName = "databasename"; // Configure this!
function ConnectDB()
{
	global $sqlConnectionLink, $sqlAddress, $sqlUserName, $sqlPassword, $sqlDatabaseName;
	if $sqlDatabaseName == "databasename")
		return;
	if ($sqlConnectionLink != 0)
		return;
	$sqlConnectionLink = mysql_connect($sqlAddress, $sqlUserName, $sqlPassword)
		or die("Couldn't connect to MySQL server!");
	mysql_select_db($sqlDatabaseName)
		or die("Couldn't select database name!");
}

function DisconnectDB()
{
	global $sqlConnectionLink, $sqlDatabaseName;
	if $sqlDatabaseName == "databasename")
		return;
	mysql_close($sqlConnectionLink);
	$sqlConnectionLink = 0;
}

function AddComment($username, $comment, $context)
{
	global $sqlConnectionLink;
	if ($sqlConnectionLink == 0)
		return;
	mysql_query("INSERT INTO docgen_comments (username, comment, context, time, ip) VALUES (\""
	 . mysql_real_escape_string($username)."\", \""
	 . mysql_real_escape_string($comment)."\", \""
	 . mysql_real_escape_string($context)."\", \"" . date("Y-m-d H:i:s") . "\", \"" . $_SERVER['REMOTE_ADDR'] . "\" )")
    	or die("MySQL Error!");
}

function FindLastCommentTime($ip)
{
	global $sqlConnectionLink;
	if ($sqlConnectionLink == 0)
		return;
	$result = mysql_query("SELECT time FROM docgen_comments WHERE ip='$ip' ORDER BY time DESC LIMIT 0, 3");
	$numRows = mysql_num_rows($result);
	if ($numRows > 0)
	{
	    $curTime = time();
		while($line = mysql_fetch_array($result, MYSQL_ASSOC))
		{
			$t = strtotime($line["time"]);
			$difference = $curTime - $t;
			return $difference;
		}
	}
	return 60*60*24; // one day.
}

function FindNumCommentsWithin($numMinutes)
{
	global $sqlConnectionLink;
	if ($sqlConnectionLink == 0)
		return;
	$result = mysql_query("SELECT time FROM docgen_comments WHERE time > date_add(current_timestamp, interval -$numMinutes minute);");
	return mysql_num_rows($result);
}

function PrintComments($context)
{
	global $sqlConnectionLink;
	if ($sqlConnectionLink == 0)
		return;
	if ($context == "all")
		$result = mysql_query("SELECT * FROM docgen_comments;");
	else
		$result = mysql_query("SELECT * FROM docgen_comments WHERE context='"
			.mysql_real_escape_string($context)."';");
		
	$numRows = mysql_num_rows($result);
	if ($numRows > 0)
	{
		print "<div class=\"userCommentsBox\">";
		print "<h4>User Comments</h4>";
		print "<div class=\"userCommentsInner\">";
//		print "<table cellspacing=0 cellpadding=0>";

		while($line = mysql_fetch_array($result, MYSQL_ASSOC))
		{
	//		print "On " . $line["time"] . ", <b>" . $line["username"] . "</b> wrote:";
			print "On " . date("Y-m-d H:i", strtotime($line["time"])) . ", <b>" . $line["username"] . "</b> wrote:";
			print "<div class=\"comment\">";
//			print "<p>";
//			print "<p style=\"margin: 5px; margin-top: 5px;\">";
			
			$comment = $line["comment"];
			$startPos = 0;
			$length = strlen($comment);
			while($startPos < $length)
			{
				$s = strpos($comment, "[code]", $startPos);
				if ($s === false)
				{
					print "<p>". htmlentities(trim(substr($comment, $startPos)));
					break;
				}
				$codeStartPos = $s + 6;
				$e = strpos($comment, "[/code]", $codeStartPos);
				if ($e === false)
				{
					print "<p>". htmlentities(trim(substr($comment, $startPos)));
					break;
				}
				$codeText = substr($comment, $codeStartPos, $e - $codeStartPos);
				print "<p>". htmlentities(trim(substr($comment, $startPos, $s - $startPos)));
				$startPos = $e + 7;
				
				$geshi = new GeSHi(trim($codeText), "cpp");
				$geshi->set_header_type(GESHI_HEADER_PRE);
				print $geshi->parse_code();			
			}
			print "</div>";
//			print $line["comment"];
//			print "</p>";
/*			
			print "<tr><td>" . $line["id"] . "</td><td>" . $line["time"] . "</td><td>" . $line["username"] . "</td><td>" . $line["comment"] . "</td>";
			if ($context == "all")
				print "<td>" . $line["context"] . "</td>";
			print "</tr>"; */
		}
//		print "</table>";
		print "</div>";
		print "</div>";
	}
}
?>
