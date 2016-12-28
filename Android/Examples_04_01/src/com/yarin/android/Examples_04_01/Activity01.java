package com.yarin.android.Examples_04_01;

import android.app.Activity;
import android.net.Uri;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ProgressBar;
import android.widget.Toast;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Iterator;

/**
 * 
 * 控制項事件透過設定其控制項的監聽器來監聽並處理事件
 * 按鍵按下事件：透過重新定義onKeyDown方法
 * 按鍵彈起事件：透過重新定義onKeyUp方法
 * 觸筆點擊事件：透過實現onTouchEvent方法
 * 範例中使用了Toast控制項：
 * Toast.makeText(this, string, Toast.LENGTH_SHORT).show();
 * 顯示提示資訊
 *
 */

public class Activity01 extends Activity
{
	// UsbDevice device;
	// UsbManager mUsbManager;	
	Uri uri;	
	long file_size;
	long file_checksum;
	 long total_checksum;
	 EditText m_EditText;
	 long g_packno; 
	 UsbDevice device = null;
	 byte[] sendbuf = new byte[64];//send host out buffer
	 byte[] rcvbuf=new byte[64];//device report buffer
	 long gcksum;
	 UsbEndpoint endpointout;
	 UsbEndpoint endpointin;
	 UsbDeviceConnection connection;
	 ProgressBar m_ProgressBar;
	 long count;
	protected static final int GUI_STOP_NOTIFIER = 0x108;
	protected static final int GUI_THREADING_NOTIFIER = 0x109;
	protected static final int GUI_STOP_FALSE = 0x110;
	
	public void onCreate(Bundle savedInstanceState)
	{
		
		super.onCreate(savedInstanceState);
		
		setContentView(R.layout.main);		
		m_ProgressBar=(ProgressBar) findViewById(R.id.progressBar1);
		m_ProgressBar.setVisibility(View.VISIBLE); //user can see it
		m_EditText=(EditText) findViewById(R.id.editText1);
		m_EditText.setKeyListener(null); //disable key input
		UsbManager mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);		

		HashMap<String, UsbDevice> deviceList = mUsbManager.getDeviceList();
		Log.d("1", deviceList.size()+" USB device(s) found");
		Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();
		
		while(deviceIterator.hasNext()){
			device= deviceIterator.next();
			Log.d("1","" + device);
		}	
		
		UsbInterface intf = device.getInterface(0);
    	boolean forceClaim = true;
    	endpointout = intf.getEndpoint(1);//it is out 
    	endpointin = intf.getEndpoint(0);//it is out 
    	connection = mUsbManager.openDevice(device);
    	connection.claimInterface(intf, forceClaim);
    	
		Button button_file = (Button) findViewById(R.id.binfile);
		button_file.setOnClickListener(new Button.OnClickListener() {
			public void onClick(View v)
			{
				Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
            	File mFile = new File("/sdcard");
            	intent.setData(Uri.fromFile(mFile));
            	intent.setType("file:///");
            	Intent Chooser = Intent.createChooser(intent, null);
            	startActivityForResult(Chooser, 0);	  	
			}
		}
	);
		//獲得Button對像
		Button button_ok = (Button) findViewById(R.id.ok);
		//設定Button控制項監聽器
		button_ok.setOnClickListener(new Button.OnClickListener() {
			public void onClick(View v)
			{
				
				if(file_size==0)
				{
					return;
					
				}
				if(device != null){
					
					Button button_file1 = (Button) findViewById(R.id.binfile);
					Button button_ok1 = (Button) findViewById(R.id.ok);
					button_file1.setEnabled(false);
					button_ok1.setEnabled(false);
					
					m_ProgressBar.setMax((int)file_size);
					m_ProgressBar.setProgress(0);
					m_ProgressBar.setVisibility(View.VISIBLE); 
					g_packno=1;
					total_checksum=0;
					
					if (CmdSyncPackno()==1)
					{
					m_EditText.append("DEVICE_ID: 0x"+Long.toHexString(CmdGetDeviceID()));
					m_EditText.append("\n");
					m_EditText.append("Erase APROM");
					m_EditText.append("\n");
					
					new Thread(new Runnable() {
						public void run()
						{
							   
						
							CmdUpdateAprom();			
										 																								
									
								
							
								
							   
					
						}
					}).start();	

		
					}
	
               }
				
			}
		}
		);

	}

	@Override
	protected void onStart() {
		// TODO Auto-generated method stub
		super.onStart();
	}

	@Override
	protected void onDestroy() {
		// TODO Auto-generated method stub
		super.onDestroy();
	
	}

	

	/* 顯示Toast  */
	public void DisplayToast(String str)
	{
		Toast.makeText(this, str, Toast.LENGTH_SHORT).show();
	}
protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        
        // TODO Auto-generated method stub
        super.onActivityResult(requestCode, resultCode, data);
        
        // 有選擇檔案
        if ( resultCode == RESULT_OK )
        {
            // 取得檔案的 Uri
        	uri= data.getData();
            
            if( uri != null )
            {

            	DisplayToast(uri.getPath());
 
            	try{
            	
            		InputStream is = new FileInputStream(uri.getPath());
            		 byte binarys[] = new byte[54];
            		 file_size = 0;
            		 long t=0;
            		 file_checksum=0;            		          
            		             		 
            	        while((t=is.read(binarys))>0)
            	        {
            	        	for(int i=0; i<t; i++)
            	            {            	   
            	        		file_checksum= file_checksum+((byte)binarys[i]&0xff);	
            	            }
            	        	 Log.d("1","t count:"+t);
            	        	 file_size+=t;            	        	
            	        }
            	        Log.d("1","size"+file_size);
            	        m_EditText.append("File Path:"+uri.getPath());
            	        m_EditText.append("\n");
            	        m_EditText.append("File Size:"+file_size);       
            	        m_EditText.append("\n");
            	        m_EditText.append("Checksum: 0x"+Long.toHexString(file_checksum));
            	        m_EditText.append("\n");
            } catch (FileNotFoundException e) {
            	// TODO Auto-generated catch block
            	e.printStackTrace();
            	}
            	catch(IOException e)
            	{            		
            		e.printStackTrace();	
            	}
            	
            }
            else
            {
                //setTitle("無效的檔案路徑 !!");
            	DisplayToast("無效的檔案路徑 !!");
            }
            
        }
        else
        {
            setTitle("取消選擇檔案 !!");
        }
    }




//NUVTON PROTOCOL IMPLEMENT
void WordsCpy(byte[] dest,byte[] src,int size,int offset)
{
    for(int i=0;i<size;i++)
    	dest[i+offset] = src[i]; 
}

void WordstoBuffer(byte[] dest,long temp)
{    
    	dest[0] = (byte)(temp>>0&0xff);
    	dest[1] = (byte)(temp>>8&0xff);
    	dest[2] = (byte)(temp>>16&0xff);
    	dest[3] = (byte)(temp>>24&0xff);
    	
}

long BuffertoWord(byte[] src,int offset)
{    
	    long temp=0;
    	temp = ((byte)src[offset+0])&0xff;
    	temp |= ((byte)src[offset+1]<<8)&0xff00;
    	temp |= ((byte)src[offset+2]<<16)&0xff0000;
    	temp |= ((byte)src[offset+3]<<24)&0xff000000;
    	return temp;
    	
}

long Checksum (byte[] src, int len)
{  
    long temp=0;

    for (int i=0; i < len; i++) {
    	temp= temp+((byte)src[i]&0xff);	
    }
    return temp;
}

void SendData()
{
	gcksum = Checksum(sendbuf, 64)&0xffffffff;
	connection.bulkTransfer(endpointout, sendbuf, 64, 10000);
}

int RcvData()
{
  int temp;
  connection.bulkTransfer(endpointin,  rcvbuf, 64, 10000);
  long lcksum=BuffertoWord(rcvbuf,0);
  long r_packno=BuffertoWord(rcvbuf,4);
  Log.d("1","lcksum: " + Long.toHexString(lcksum));
  Log.d("1","r_packno: " + r_packno);
  if(lcksum!=gcksum)
  {
	  temp=0;
  }
  else
  {
	  if(r_packno!=g_packno)
	  {		 
	  temp=0;	  
	  }
	  else
	  {
		  temp=1;
		  g_packno++;
	  }
  }
  return temp;
}

int CmdSyncPackno()
{	
    byte temp[]= new byte[4];
	//initial send buffer all zero.
	for(int i=0;i<64;i++)
	{		
		sendbuf[i]=0;
	}
	
	//define CMD_SYNC_PACKNO		0x000000A4;
	WordstoBuffer(temp,0x000000A4);
	WordsCpy(sendbuf,temp,4,0);
	WordstoBuffer(temp,g_packno);
	WordsCpy(sendbuf,temp,4,4);
	WordsCpy(sendbuf,temp,4,8);
	
	g_packno++;
	SendData();
	int ret=RcvData();
	if(ret==0)
	{		
		  Log.d("1","error!!");
		  return 0;
	}
	return 1;
}

long CmdGetDeviceID()
{	
    byte temp[]= new byte[4];
	//initial send buffer all zero.
	for(int i=0;i<64;i++)
	{		
		sendbuf[i]=0;
	}
	
//	CMD_GET_DEVICEID	0x000000B1
	WordstoBuffer(temp,0x000000B1);
	WordsCpy(sendbuf,temp,4,0);
	WordstoBuffer(temp,g_packno);
	WordsCpy(sendbuf,temp,4,4);
	
	g_packno++;
	SendData();
	int ret=RcvData();
	if(ret==0)
	{		
		  Log.d("1","error!!");
	}
	long DEVICE_ID=BuffertoWord(rcvbuf,8);
	return DEVICE_ID;
   }

int CmdUpdateAprom()
{	
	
	//load file to program	
	 try{
    byte temp[]= new byte[4];
    int ret;
	//initial send buffer all zero.
	for(int i=0;i<64;i++)
	{		
		sendbuf[i]=0;
	}
	
	//#define CMD_UPDATE_APROM	0x000000A0
	WordstoBuffer(temp,0x000000A0);
	WordsCpy(sendbuf,temp,4,0);
	WordstoBuffer(temp,g_packno);
	WordsCpy(sendbuf,temp,4,4);		
	g_packno++;
	
	//start address
	long startaddr = 0;
	WordstoBuffer(temp,startaddr);
	WordsCpy(sendbuf,temp,4,8);
	
	WordstoBuffer(temp,file_size);
	WordsCpy(sendbuf,temp,4,12);

	
	
	
	InputStream is = new FileInputStream(uri.getPath());
	byte binarys[] = new byte[56];
	
	long file_count=0;	
	byte binarys1[] = new byte[48];
	file_count=is.read(binarys1);
	Log.d("1","file_count:"+file_count);
	WordsCpy(sendbuf,binarys1,48,16);
	
	SendData();           
	Thread.currentThread();
	Thread.sleep(1000);//delay 2000ms for MCU erase aprom
	ret=RcvData();
	if (ret==0)  //try agent 
	ret=RcvData();
	if(ret==0)
	{		
		  Log.d("1","error!!");
		  return 0;
	}
	
	for ( count = 48; count < file_size; count = count + 56)
	{
		Message m = new Message();
		m.what = Activity01.GUI_THREADING_NOTIFIER;
		Activity01.this.myMessageHandler.sendMessage(m);	 
      file_count=is.read(binarys);    
      for(int i=0;i<64;i++)
  	  {		
  		sendbuf[i]=0;
  	  }
  	  WordstoBuffer(temp,g_packno);
  	  WordsCpy(sendbuf,temp,4,4);		
  	  g_packno++;
  	  if ((file_size - count) > 56)
	  {	
  		WordsCpy(sendbuf,binarys,56,8);
  		SendData();
  		ret=RcvData();
  		if(ret==0)
  		{		
  			 m = new Message();

		    	m.what = Activity01.GUI_STOP_FALSE;
		    	Activity01.this.myMessageHandler.sendMessage(m);
  			return 0;
  		}
	  }else{
			WordsCpy(sendbuf,binarys,(int)(file_size-count),8);
	  		SendData();
	  		ret=RcvData();
	  		if(ret==0)
	  		{	
	  			 m = new Message();

			    	m.what = Activity01.GUI_STOP_FALSE;
			    	Activity01.this.myMessageHandler.sendMessage(m);
	  			return 0;
	  		}
		 
		    total_checksum=BuffertoWord(rcvbuf,8);
		    if((file_checksum&0xffff)!=(total_checksum&0xffff))
		    {
		    	Log.d("1","cecheck: 0x"+Long.toHexString(total_checksum));
		    	Log.d("1","DE: 0x"+Long.toHexString(file_checksum));
		    	 m = new Message();

		    	m.what = Activity01.GUI_STOP_FALSE;
		    	Activity01.this.myMessageHandler.sendMessage(m);
		    	return 0;
		    }
	       }
  	  
	}
	Message m = new Message();

	m.what = Activity01.GUI_STOP_NOTIFIER;
	Activity01.this.myMessageHandler.sendMessage(m);
	
} catch (FileNotFoundException e) {
e.printStackTrace();
}
catch(IOException e)
{            		
	e.printStackTrace();	
}
 catch (InterruptedException e) {
	 Log.d("1","error!!");   	   
 }


	return 1;
}

Handler myMessageHandler = new Handler()
{
  // @Override 
	  public void handleMessage(Message msg)
	  {
		  switch (msg.what)
		  {
		  //ProgressBar已經是對大值
		  case Activity01.GUI_STOP_NOTIFIER:
			  //m_ProgressBar.setVisibility(View.GONE);
			  
				m_ProgressBar.setVisibility(View.VISIBLE); 
			  Thread.currentThread().interrupt();
			  m_EditText.append("Program ok");
				 m_EditText.append("\n");
				 m_EditText.append("Checksum ok");
				 m_EditText.append("\n");
					Button button_file1 = (Button) findViewById(R.id.binfile);
					Button button_ok1 = (Button) findViewById(R.id.ok);
					button_file1.setEnabled(true);
					button_ok1.setEnabled(true);
			  break;
			  
		  case Activity01.GUI_STOP_FALSE:
			  //m_ProgressBar.setVisibility(View.GONE);			                			
			  Thread.currentThread().interrupt();
			  
				 m_EditText.append("\n");
				 m_EditText.append("Check False");
				 m_EditText.append("\n");
			  break;  
			  
		  case Activity01.GUI_THREADING_NOTIFIER:
			  
				  // 改變ProgressBar的目前值
			      m_ProgressBar.setVisibility(View.VISIBLE); 
				  m_ProgressBar.setProgress((int) count);				  
				  
				  // 設定標題欄中前景的一個進度條進度值
				  setProgress((int) count);
			  
			  break;
		  }
		  super.handleMessage(msg);
	 }
};


	
}
