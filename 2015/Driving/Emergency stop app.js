//written with androidscript

//Vars
var stop = 0;
var str  = 0.5;
var spd  = 0.5;
var record = false;
const slow = 0.5;
const fast = 2.0;

//Called when application is started.
function OnStart()
{
	//Prevent screen from dimming or locking while using app
	app.PreventScreenLock( true );

	//Create a layout with objects vertically centered.
	lay = app.CreateLayout( "linear", "VCenter,FillXY" );

	//Create a button 1/3 of screen width and 1/4 screen height.
	btn = app.CreateButton( "Connect", 0.4, 0.15 );
	btn.SetOnTouch( btn_Connect_OnTouch );
	lay.AddChild( btn );

	//Create stop button
	btnGoStop = app.CreateButton( "STOP!", 0.7, 0.35, "Lego");
	btnGoStop.SetOnTouch( tgl_OnTouch );
	lay.AddChild( btnGoStop );

	//Create recording toggle button
	tglRec = app.CreateToggle( "Record", 0.4, 0.15);
	tglRec.SetOnTouch( recOnTouch );
	lay.AddChild( tglRec );

	//Create speed toggle button
	tglSpd = app.CreateToggle( "Toggle Speed", 0.4, 0.15 );
	tglSpd.SetOnTouch( spdOnTouch );
	lay.AddChild( tglSpd );

	//Steering slider
	skb = app.CreateSeekBar( 0.9 );
	skb.SetRange( 1.0 );
	skb.SetValue( 0.5 );
	skb.SetOnTouch( skb_OnTouch );
	lay.AddChild( skb );

	//Create layout
	app.AddLayout( lay );

	//Create Bluetooth serial object.
	bt = app.CreateBluetoothSerial();
	bt.SetOnConnect( bt_OnConnect );
	//bt.SetOnReceive( bt_OnReceive );
	// bt.Connect( "SerialBT" );
	bt.SetSplitMode( "End", "\n" );

}

//Called when user touches the button.
function btn_Connect_OnTouch()
{
    bt.Connect( "SerialBT" );
}

//Called when toggle button is touched
function spdOnTouch( isChecked )
{
	if (isChecked) {
	    speed = fast;
	    app.ShowPopup("Gotta go fast!");
	}
	else {
	    spd = slow;
	    app.ShowPopup("Woah boy!");
	}
}

//Called when we are connected.
function bt_OnConnect( ok )
{
	if ( ok ) setTimeout( communicate, 100 );
	else app.ShowPopup( "Failed to connect!" );
}

//Called when user touches our toggle button.
function tgl_OnTouch()
{
	if(stop)
	{
		stop = 0;
		btnGoStop.SetText( "STOP!" );
	}
	else
	{
		stop = 1;
		btnGoStop.SetText( "GO!" );
	}
}

//Called when the user touches the toggle recoring button
function recOnTouch( isChecked )
{
	record = isChecked;
	if (isChecked)
	    app.ShowPopup("Starting recording...");
	else
	    app.ShowPopup("Terminating recording...");
}

//Slider touch function
function skb_OnTouch( value )
{
	str = value;
	// Map to something other than 0 - 1?
	app.ShowPopup("Steering angle: " + str);
}

//Timed comunication function
function communicate()
{
	bt.Write( stop + "," + spd + "," + str + "," + record + "\n");
	setTimeout( communicate, 100 ); // Call self again in 100 ms
}
