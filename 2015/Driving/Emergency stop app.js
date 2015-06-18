//written with androidscript

//Consts
const slow = 2;
const fast = 10;

//Vars
var stop = 0;
var str  = 0.5;
var spd  = slow;
var record = 0;

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
	    spd = fast;
	    app.ShowPopup("Gotta go fast! Speed is: " + spd);
	}
	else {
	    spd = slow;
	    app.ShowPopup("Woah boy! Speed is: " + spd);
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
    app.ShowPopup("<" + "3," + (0+record)+ "," +(3+record)+ ">");
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
	// Map to 0 - 180
	str = Math.floor(str * 180);
	app.ShowPopup("Steering angle: " + str);
}

//Timed comunication function
function communicate()
{
	//              adr    val           checksum
	bt.Write( "<" + "0," + stop   + ","+ (0+stop) + ">");
	bt.Write( "<" + "1," + spd    + ","+ (1+spd)  + ">");
	bt.Write( "<" + "2," + stop   + ","+ (2+stop) + ">");
	bt.Write( "<" + "3,"+(0+record)+","+(3+record)+ ">");
	//0+record is a hack to force the bool to get sent as 0 or 1

	setTimeout( communicate, 100 ); // Call self again in 100 ms
																	// Faster?
}
