"""
Multi-Agent System - Entry Point

Khởi tạo và chạy pipeline Multi-Agent cho code conversion & review.
"""

# from crewai import Crew, Process
# from agents import code_converter, syntax_verifier, logic_verifier, chief_reviewer
# from tasks import convert_task, syntax_check_task, logic_check_task, review_task


def run_pipeline(input_code: str, source_lang: str, target_lang: str):
    """
    Chạy pipeline chuyển đổi và review code.

    Args:
        input_code: Code nguồn cần chuyển đổi
        source_lang: Ngôn ngữ nguồn (e.g., "C++")
        target_lang: Ngôn ngữ đích (e.g., "Python")
    """
    # TODO: Implement khi có API key
    # crew = Crew(
    #     agents=[code_converter, syntax_verifier, logic_verifier, chief_reviewer],
    #     tasks=[convert_task, syntax_check_task, logic_check_task, review_task],
    #     process=Process.sequential,
    #     verbose=True,
    # )
    # result = crew.kickoff(inputs={
    #     "code_input": input_code,
    #     "source_lang": source_lang,
    #     "target_lang": target_lang,
    # })
    # return result
    print("Pipeline chưa được cấu hình. Vui lòng setup API key trong .env")


if __name__ == "__main__":
    sample_code = """
    // Sample Arduino code
    void setup() {
        Serial.begin(9600);
    }
    void loop() {
        Serial.println("Hello");
        delay(1000);
    }
    """
    run_pipeline(sample_code, "C++", "Python")
