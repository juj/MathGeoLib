<?php 
session_start();
date_default_timezone_set("Europe/Helsinki");

if($_SESSION["captcha"]!=$_POST["captcha"])
{
	print "<html><body>Invalid captcha!<br /><a href=\"".$_POST["redirectBack"]."\">Go back</a></body</html>";
	die();
}

include "comments.php";

ConnectDB();

$timeSinceLastComment = FindLastCommentTime($_SERVER['REMOTE_ADDR']);
if ($timeSinceLastComment < 60 * 5) // Must wait 5 minutes between comments.
{
	print "<html><body>You must wait 5 minutes between comments!<br /><a href=\"".$_POST["redirectBack"]."\">Go back</a></body</html>";
	die();
}
$numComments = FindNumCommentsWithin(60);
if ($numComments > 10)
{
	print "<html><body>Too many comments posted in short time period!<br /><a href=\"".$_POST["redirectBack"]."\">Go back</a></body></html>";
	die();
}
if (strlen($_POST["username"]) > 0 && strlen($_POST["comments"]) > 0 && strlen($_POST["context"]) > 0)
	AddComment($_POST["username"], $_POST["comments"], $_POST["context"]);
else
{
	print "<html><body>Must have both an username and comment to send!<br /><a href=\"".$_POST["redirectBack"]."\">Go back</a></body</html>";
	die();
}

//  PrintComments("all");
DisconnectDB();

header('Location: ' . $_POST["redirectBack"] . "#comments");
?>
