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
	{
            "name": "open gripper",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.OpenGripper",
                "trigger": {
                    "queryPatterns": [
                        "open gripper"
                    ]
                }
            },
            "fulfillment": {
                "staticFulfillment": {
                    "templatedResponse": {
                        "items": [
                            {
                                "simpleResponse": {
                                    "textToSpeech": "Ok, I am ready to pick up any object now!"
                                }
                            },
                            {
                                "deviceExecution": {
                                    "command": "OpenGripper"
                                }
                            }
                        ]
                    }
                }
            }
        },
	{
            "name": "close gripper",
            "availability": {
                "deviceClasses": [
                    {
                        "assistantSdkDevice": {}
                    }
                ]
            },
            "intent": {
                "name": "com.example.intents.CloseGripper",
                "trigger": {
                    "queryPatterns": [
                        "close gripper"
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
                                    "command": "CloseGripper"
                                }
                            }
                        ]
                    }
                }
            }
        },
         ]
            }
        }
    }
}
```
## Deploy the Action Package
Now make the Action Package accessible to the Google Assistant Server.

While you can do the steps in this section on the device, it may be easier to do them on your development machine. These commands do not require a virtual environment to run.

1. [Download](https://developers.google.com/actions/tools/gactions-cli) the ```gactions``` command line tool
	* Note: On Linux, run chmod +x gactions to make the downloaded binary executable.
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
