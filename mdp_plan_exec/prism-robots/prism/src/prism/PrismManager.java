//==============================================================================
//	
//	Copyright (c) 2002-
//	Authors:
//	* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//	
//------------------------------------------------------------------------------
//	
//	This file is part of PRISM.
//	
//	PRISM is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//	
//	PRISM is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//	
//	You should have received a copy of the GNU General Public License
//	along with PRISM; if not, write to the Free Software Foundation,
//	Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//	
//==============================================================================

package prism;

import java.io.*;
import java.net.*;
import java.util.*;

import parser.ast.*;

/**
* Example class demonstrating how to control PRISM programmatically,
* i.e. through the "API" exposed by the class prism.Prism.
* (this now uses the newer version of the API, released after PRISM 4.0.3)
* Test like this:
* PRISM_MAINCLASS=prism.PrismTest bin/prism ../prism-examples/polling/poll2.sm ../prism-examples/polling/poll3.sm
*/
public class PrismManager
{
   
    private Prism prism;
    private String currentTimeOfDay;
    private Map<String,ModulesFile> mdpModels;
    private PropertiesFile specification;
    private ServerSocket server;
    String directory;

    
    
    public PrismManager(){
        try{
            PrismLog mainLog;
            
            //init socket
            server = new ServerSocket(8085);
            System.out.println("PRISM server running on port 8085");
            

            mdpModels = new HashMap<String,ModulesFile>();
            
            directory=System.getProperty("user.home")+"/tmp/prism/";
                        
            // Init PRISM
            mainLog = new PrismFileLog("stdout");
            prism = new Prism(mainLog, mainLog);
            prism.initialise();
        }
        catch (PrismException e) {
            System.out.println("Error: " + e.getMessage());
        }
        catch (IOException e) {
            System.out.println("Error: " + e.getMessage());
        }
    }
        
    
    
    public Prism getPrism(){
        return prism;
    }
    
    public String getCurrentTimeOfDay(){
        return currentTimeOfDay;
    }
    
    public Map<String,ModulesFile> getMdpModels(){
        return mdpModels;
    }
    
    public PropertiesFile getSpecification(){
        return specification;
    }
       
              
    public ServerSocket getServer(){
        return server;
    }
    
    
    public void setCurrentModel(String timeOfDay){
        if (! timeOfDay.equals(currentTimeOfDay)){
            currentTimeOfDay=timeOfDay;
            ModulesFile model = mdpModels.get(timeOfDay);
            prism.loadPRISMModel(model);
        }
    }
    
    public boolean addMdpModel(String timeOfDay,String mdpTextFile){
        try{
            if (mdpModels.containsKey(timeOfDay)){
                return false;
            }
            mdpModels.put(timeOfDay,prism.parseModelFile(new File(mdpTextFile)));
            return true;
        }
        catch (FileNotFoundException e) {
            System.out.println("Error: " + e.getMessage());
            return false;
        }
        catch (PrismException e) {
            System.out.println("Error: " + e.getMessage());
            return false;
        }
    }
    
    
    public Result callPrism(String timeOfDay,String ltlTask, boolean generatePolicy)  {
        try {
            ModulesFile model;
            PropertiesFile propertiesFile;
            Result result;
            
            
            if (! timeOfDay.equals(currentTimeOfDay)){
                setCurrentModel(timeOfDay);
            }
            
            model = mdpModels.get(timeOfDay);
            
            if(generatePolicy){
                prism.getSettings().set(PrismSettings.PRISM_EXPORT_ADV, "DTMC");
                prism.getSettings().set(PrismSettings.PRISM_EXPORT_ADV_FILENAME,directory + timeOfDay + "/adv.tra");
                prism.setExportProductStates(true);
                prism.setExportProductStatesFilename(directory + timeOfDay + "/prod.sta");
                prism.setExportProductTrans(true);
                prism.setExportProductTransFilename(directory + timeOfDay + "/prod.tra");
                prism.setExportTarget(true);
                prism.setExportTargetFilename(directory + timeOfDay + "/prod.lab");
            } else {
                prism.getSettings().set(PrismSettings.PRISM_EXPORT_ADV, "None");
                prism.setExportProductStates(false);
                prism.setExportProductTrans(false);
                prism.setExportTarget(false);
            }
            

            propertiesFile = prism.parsePropertiesString(model, ltlTask);
            result = prism.modelCheck(propertiesFile, propertiesFile.getPropertyObject(0));
            System.out.println(result.getResult());
            return result;

        }
        catch (PrismException e) {
            System.out.println("Error: " + e.getMessage());
            return null;
        }
    }
    
    public static void main(String args[]) throws Exception {
            
            
        String command;
        String timeOfDayCommand;
        String modelFile;
        String auxReader;
        String toClient;
        Socket client;
        String specification;
        
        boolean isNewTimeOfDay;
        
        PropertiesFile propertiesFile;
        Result result;

            
        PrismManager manager=new PrismManager();
        
        
            
        client = manager.server.accept();
        System.out.println("got connection on port 8085");   
        boolean run = true;
        
        while(run) {
           
            BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
            PrintWriter out = new PrintWriter(client.getOutputStream(),true);
            
            command = in.readLine();
            System.out.println("received: " + command);
            
            
            if(command == null){
                client = manager.server.accept();
                System.out.println("got connection on port 8085");
            } else {            
                if(command.equals("add")) {
                    
                    System.out.println("adding");
                    timeOfDayCommand=in.readLine();
                    System.out.println("time of day: " + timeOfDayCommand);
                    modelFile=in.readLine(); 
                    isNewTimeOfDay=manager.addMdpModel(timeOfDayCommand,modelFile);
                    if (isNewTimeOfDay){
                        toClient = "added";
                        System.out.println("added");
                        out.println(toClient);
                    } else {
                        toClient = "repeated";
                        System.out.println("repeated");
                        out.println(toClient);
                    }
                }
                if (command.equals("check")){
                    System.out.println("checking");
                    timeOfDayCommand=in.readLine();
                    manager.setCurrentModel(timeOfDayCommand);
                    System.out.println("time of day: " + timeOfDayCommand);
                    specification=in.readLine();
                   // propertiesFile = manager.prism.parsePropertiesString(manager.mdpModels.get(timeOfDayCommand), specification);
                   // result = manager.prism.modelCheck(propertiesFile, propertiesFile.getPropertyObject(0));
                    result=manager.callPrism(timeOfDayCommand,specification,false);
                    System.out.println(result.getResult());
                    toClient = result.getResult().toString();
                    System.out.println("checked");
                    out.println(toClient);
                }
                 if (command.equals("plan")){
                    System.out.println("planning");
                    timeOfDayCommand=in.readLine();
                    manager.setCurrentModel(timeOfDayCommand);
                    System.out.println("time of day: " + timeOfDayCommand);
                    specification=in.readLine();
                    //propertiesFile = manager.prism.parsePropertiesString(manager.mdpModels.get(timeOfDayCommand), specification);
                    //result = manager.prism.modelCheck(propertiesFile, propertiesFile.getPropertyObject(0));
                    result=manager.callPrism(timeOfDayCommand,specification,true);
                    System.out.println(result.getResult());
                    toClient = "planned";
                    System.out.println("planned");
                    out.println(toClient);
                }
                if (command.equals("shutdown")){
                    run=false;
                    client.close();
                    manager.server.close();
                    manager.prism.closeDown();
                    
                }
            }
                    
        }       

        
        System.exit(0);
    }
}








                
