"""
Multi-Agent System - Agent Definitions

Sử dụng CrewAI để định nghĩa các AI Agents cho pipeline
code conversion, verification và review.
"""

# from crewai import Agent
# from langchain_openai import ChatOpenAI

# TODO: Uncomment và cấu hình khi có API key

# llm = ChatOpenAI(model="gpt-4", temperature=0.1)

# code_converter = Agent(
#     role="Senior Software Engineer",
#     goal="Chuyển đổi code giữa các ngôn ngữ, giữ nguyên logic gốc",
#     backstory="Bạn là kỹ sư phần mềm senior với 15 năm kinh nghiệm đa ngôn ngữ.",
#     llm=llm,
#     verbose=True,
# )

# syntax_verifier = Agent(
#     role="Build Engineer",
#     goal="Kiểm tra code có đúng cú pháp không",
#     backstory="Bạn là chuyên gia kiểm tra build và syntax với kinh nghiệm sâu về compiler.",
#     llm=llm,
#     verbose=True,
# )

# logic_verifier = Agent(
#     role="QA Automation Engineer",
#     goal="Đảm bảo tính tương đồng logic giữa code gốc và code mới",
#     backstory="Bạn là kỹ sư QA chuyên về phân tích logic và viết test tự động.",
#     llm=llm,
#     verbose=True,
# )

# chief_reviewer = Agent(
#     role="Tech Lead",
#     goal="Đánh giá chất lượng code tổng thể và đưa ra báo cáo review",
#     backstory="Bạn là Tech Lead với tầm nhìn kiến trúc và tiêu chuẩn code cao.",
#     llm=llm,
#     verbose=True,
# )
