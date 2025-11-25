import argparse
from concept_creator.concept_agent import generate_concept
from test_writer.test_writer_agent import write_tests

def main():
    parser = argparse.ArgumentParser(description="AI Agent Pipeline")
    parser.add_argument("--concept", action="store_true", help="Generate test concept only")
    parser.add_argument("--tests", action="store_true", help="Generate tests from existing concept.json")
    parser.add_argument("--full", action="store_true", help="Run full pipeline: concept → tests")

    args = parser.parse_args()

    if args.concept:
        print("📘 Generating test concept...")
        generate_concept()
        return

    if args.tests:
        print("🧪 Generating tests from concept.json...")
        write_tests()
        return

    if args.full:
        print("🚀 Running full pipeline (concept + tests)...")
        concept = generate_concept()
        # write_tests()
        return

    parser.print_help()

if __name__ == "__main__":
    main()
