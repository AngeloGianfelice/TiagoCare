import argparse
from source.tiago_node import TiagoNode
from source.ska_pipeline import SKA_pipeline
import rospy

def main():
    parser = argparse.ArgumentParser(description="2 modes script")

    parser.add_argument("--mode", type=str, choices=["interaction","ska"], required = True, help="Mode to run: interaction or semantic knowledge agent(ska) module")
    parser.add_argument("--log_dir", type=str, default='new_task',
                        help="logging directory for new task")
    parser.add_argument("--task", type=str, default="task1",
                        help="Task chosen (task1, task2, task3)")
    parser.add_argument("--reasoning", type=str, default="true",
                        help="Use reasoning (true | false)")
    parser.add_argument("--model", type=str, default="gemini",
                        help="Use gemini, deepseek or llama")
    args = parser.parse_args()

    rospy.init_node("tiago_node")
    
    if args.mode == 'interaction':
        # check required params for tiagocare interaction
        if not all([args.task, args.reasoning, args.model]):
            parser.error("Mode interaction requires --task, --reasoning, and --model")
        try:
            print(f"\n######## Chosen tiagocare parameters ########\n task: {args.task}\n reasoning: {args.reasoning}\n model: {args.model}")
            node = TiagoNode(args.task, args.reasoning, args.model)
            node.run()
        except rospy.ROSInterruptException:
            pass

    elif args.mode == 'ska':
        if not args.log_dir:
            parser.error("Mode ska requires --log_dir parameter")
        print(f"\n######## Chosen Semantic Knowledge Agent parameters ########\n log_dir: {args.log_dir}")
        pipeline = SKA_pipeline(args.log_dir)
        pipeline.run()

if __name__ == "__main__":
    main()
