# Register Custom On Device Actions

The following are instructions on how to set up Custom On Device Actions

## Getting Started

A Custom Device Action specifies the command that is sent to your device to trigger its special ability.
To define a Custom Device Action, you need four pieces of information:
1. A pattern to try to match against the user query
2. A Custom Device Action to associate with a matched query
3. Text that is spoken back to the user if the device supports the action
4. A command name that is sent back to the device, along with any parameters
You create the Custom Device Action by putting this information into an Actions on Google ([Action Package](https://developers.google.com/actions/reference/rest/Shared.Types/ActionPackage)). Action Packages define the format for Google Assistant Server responses. Unlike the Actions SDK, Custom Device Actions are fulfilled locally; you do not specify an endpoint to process requests and provide responses. Custom Device Actions are not conversational in nature.

### Create an Action Package

Create a file (e.g., actions.json) that defines a test command – open/close the gripper. You can use the example below:
```
{
    "manifest": {
        "displayName": "start position",
        "invocationName": "start position",
        "category": "PRODUCTIVITY"
    },
    "actions": [
        {
            "name": "start position",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.StartPosition",
                "trigger": {
                    "queryPatterns": [
                        "start position"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Ok, going back to initial position. What should I do now?"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "StartPosition"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "pick up the object",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.PickUpTheObject",
                "trigger": {
                    "queryPatterns": [
                        "pick up the object"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "ssml": "<speak> Working on it <break time =\"6\" /> Where do you want me to place it? </speak>"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "PickUpTheObject"
                                }
                            }
                        ]
                    }
                }
            }
        }, 
	
	
	{
            "name": "cylinder open",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.CylinderOpen",
                "trigger": {
                    "queryPatterns": [
                        "cylinder open"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Ok, I am ready to pick up any cylinder now!"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "CylinderOpen"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "cylinder close",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.CylinderClose",
                "trigger": {
                    "queryPatterns": [
                        "cylinder close"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Okey-dokey!"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "CylinderClose"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "cycle position",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.CyclePosition",
                "trigger": {
                    "queryPatterns": [
                        "cycle hand positions"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Sure, running a demo of the Schunk hand. This is an example of what I can do"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "CyclePosition"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "open hand",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.OpenHand",
                "trigger": {
                    "queryPatterns": [
                        "open hand"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Sure, You're the boss!"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "OpenHand"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "set push mode on",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.SetPushModeON",
                "trigger": {
                    "queryPatterns": [
                        "set push mode on"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "ssml": "<speak> Compliance mode is on, you can now push the robot around! </speak>"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "SetPushModeON"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "reset push mode",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.ResetPushMode",
                "trigger": {
                    "queryPatterns": [
                        "reset push mode"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "ssml": "<speak> I have sucessfully set compliance mode off</speak>"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "ResetPushMode"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "close hand",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.CloseHand",
                "trigger": {
                    "queryPatterns": [
                        "close hand"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "ssml": "<speak> <break time =\"1\" /> Hand now closed! </speak>"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "CloseHand"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "drop object on default location",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.DropObjectOnDefaultLocation",
                "trigger": {
                    "queryPatterns": [
                        "drop object on default location"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "ssml": "<speak> Ok I am moving the object on the x. Please let me know when it is safe for me to drop it </speak>"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "DropObjectOnDefaultLocation"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "say hi to my new friends",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.SayHiToMyNewFriends",
                "trigger": {
                    "queryPatterns": [
                        "say hi to my new friends"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "ssml": "<speak> Oh hi Gianmarco. I see you have some friends today! Would you like me to show everyone what I can do? <break time =\"3\" /> Ok, well, you designed me to facilitate Human-Robot Collaboration for the Industry 4.0. Practically I have got all the functionalities of the normal Google Assistant but I am also able to control the KUKA iiwa robot to mimic a manufacturing process that could take place in a smart factory. I can also tell you the weather, sports results and we can play some games together if you get bored. I am also able to perform a Google search if you need it, or add a new event to your Calendar. Shall we get started? </speak>"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "DropObjectOnDefaultLocation"
                                }
                            }
                        ]
                    }
                }
            }
        }
    ]
}```
## Deploy the Action Package
Now make the Action Package accessible to the Google Assistant Server.

While you can do the steps in this section on the device, it may be easier to do them on your development machine. These commands do not require a virtual environment to run.

1. [Download](https://developers.google.com/actions/tools/gactions-cli) the ```gactions``` command line tool
	* Note: On Linux, run ```chmod +x gactions``` to make the downloaded binary executable.
2. Remove any existing credentials from the same directory as the gactions tool.
```
rm creds.data
```
3. Save your Action Package to Google by using the gactions CLI. Replace project_id with your Actions Console project ID.
```
./gactions update --action_package actions.json --project project_id
```
4. The first time you run this command you will be given a URL and be asked to sign in. Ask for help if this is the case.

5. Deploy your action package into test mode by using the gactions CLI. You must have saved your Action Package to Google at least once before running this command. Test mode enables the action package on your user account only.
```
./gactions test --action_package actions.json --project project_id
```
7. To update the action package, use the gactions update command.

For more info visit:
https://developers.google.com/assistant/sdk/guides/library/python/extend/custom-actions
