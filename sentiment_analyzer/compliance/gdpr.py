"""
GDPR compliance implementation for sentiment analysis
"""

import time
import uuid
import hashlib
import json
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
import logging
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)


class DataSubjectRights(Enum):
    """GDPR data subject rights"""
    ACCESS = "access"  # Article 15
    RECTIFICATION = "rectification"  # Article 16
    ERASURE = "erasure"  # Article 17 (Right to be forgotten)
    RESTRICT_PROCESSING = "restrict_processing"  # Article 18
    DATA_PORTABILITY = "data_portability"  # Article 20
    OBJECT = "object"  # Article 21
    WITHDRAW_CONSENT = "withdraw_consent"  # Article 7


class LegalBasis(Enum):
    """GDPR legal basis for processing"""
    CONSENT = "consent"  # Article 6(1)(a)
    CONTRACT = "contract"  # Article 6(1)(b)
    LEGAL_OBLIGATION = "legal_obligation"  # Article 6(1)(c)
    VITAL_INTERESTS = "vital_interests"  # Article 6(1)(d)
    PUBLIC_TASK = "public_task"  # Article 6(1)(e)
    LEGITIMATE_INTERESTS = "legitimate_interests"  # Article 6(1)(f)


class ProcessingPurpose(Enum):
    """Purposes for processing personal data"""
    SENTIMENT_ANALYSIS = "sentiment_analysis"
    QUALITY_IMPROVEMENT = "quality_improvement"
    RESEARCH = "research"
    SECURITY = "security"
    LEGAL_COMPLIANCE = "legal_compliance"


@dataclass
class ConsentRecord:
    """Record of data subject consent"""
    consent_id: str
    data_subject_id: str
    purposes: List[ProcessingPurpose]
    timestamp: float
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
    consent_text: Optional[str] = None
    withdrawal_timestamp: Optional[float] = None
    is_active: bool = True


@dataclass
class ProcessingActivity:
    """GDPR Article 30 processing activity record"""
    activity_id: str
    purpose: ProcessingPurpose
    legal_basis: LegalBasis
    data_categories: List[str]
    data_subjects: List[str]
    recipients: List[str]
    retention_period: str
    security_measures: List[str]
    transfer_outside_eu: bool = False
    adequacy_decision: Optional[str] = None


@dataclass
class DataSubjectRequest:
    """Data subject rights request"""
    request_id: str
    data_subject_id: str
    request_type: DataSubjectRights
    timestamp: float
    status: str  # pending, verified, processed, completed
    data_provided: Optional[Dict] = None
    verification_method: Optional[str] = None
    response_timestamp: Optional[float] = None


class GDPRCompliance:
    """GDPR compliance manager for sentiment analysis"""
    
    def __init__(self, controller_name: str, dpo_contact: str):
        self.controller_name = controller_name
        self.dpo_contact = dpo_contact
        
        # Data stores (in production, these would be secure databases)
        self.consent_records: Dict[str, ConsentRecord] = {}
        self.processing_activities: Dict[str, ProcessingActivity] = {}
        self.data_subject_requests: Dict[str, DataSubjectRequest] = {}
        self.processing_log: List[Dict[str, Any]] = []
        
        # Configuration
        self.default_retention_period = 365 * 24 * 3600  # 1 year in seconds
        self.consent_refresh_period = 2 * 365 * 24 * 3600  # 2 years
        
        # Initialize default processing activities
        self._initialize_processing_activities()
        
        logger.info(f"GDPR compliance initialized for controller: {controller_name}")
    
    def _initialize_processing_activities(self):
        """Initialize default processing activities"""
        activities = [
            ProcessingActivity(
                activity_id="sentiment_analysis_core",
                purpose=ProcessingPurpose.SENTIMENT_ANALYSIS,
                legal_basis=LegalBasis.LEGITIMATE_INTERESTS,
                data_categories=["text_content", "analysis_results"],
                data_subjects=["users", "customers"],
                recipients=["internal_systems"],
                retention_period="1 year",
                security_measures=["encryption", "access_control", "audit_logging"],
                transfer_outside_eu=False
            ),
            ProcessingActivity(
                activity_id="quality_improvement",
                purpose=ProcessingPurpose.QUALITY_IMPROVEMENT,
                legal_basis=LegalBasis.LEGITIMATE_INTERESTS,
                data_categories=["analysis_results", "performance_metrics"],
                data_subjects=["users"],
                recipients=["internal_systems", "development_team"],
                retention_period="2 years",
                security_measures=["anonymization", "aggregation", "access_control"],
                transfer_outside_eu=False
            ),
            ProcessingActivity(
                activity_id="research_analytics",
                purpose=ProcessingPurpose.RESEARCH,
                legal_basis=LegalBasis.CONSENT,
                data_categories=["anonymized_text", "statistical_data"],
                data_subjects=["research_participants"],
                recipients=["research_team", "academic_partners"],
                retention_period="5 years",
                security_measures=["anonymization", "pseudonymization", "encryption"],
                transfer_outside_eu=True,
                adequacy_decision="Standard Contractual Clauses"
            )
        ]
        
        for activity in activities:
            self.processing_activities[activity.activity_id] = activity
    
    def collect_consent(self, data_subject_id: str, purposes: List[ProcessingPurpose],
                       ip_address: Optional[str] = None, user_agent: Optional[str] = None,
                       consent_text: Optional[str] = None) -> str:
        """Collect explicit consent from data subject"""
        
        consent_id = str(uuid.uuid4())
        
        consent_record = ConsentRecord(
            consent_id=consent_id,
            data_subject_id=data_subject_id,
            purposes=purposes,
            timestamp=time.time(),
            ip_address=ip_address,
            user_agent=user_agent,
            consent_text=consent_text,
            is_active=True
        )
        
        self.consent_records[consent_id] = consent_record
        
        # Log the consent collection
        self._log_processing_activity(
            "consent_collected",
            data_subject_id,
            {"consent_id": consent_id, "purposes": [p.value for p in purposes]}
        )
        
        logger.info(f"Consent collected for data subject {data_subject_id}: {consent_id}")
        return consent_id
    
    def check_consent(self, data_subject_id: str, purpose: ProcessingPurpose) -> bool:
        """Check if valid consent exists for specific purpose"""
        
        current_time = time.time()
        
        for consent_record in self.consent_records.values():
            if (consent_record.data_subject_id == data_subject_id and
                consent_record.is_active and
                purpose in consent_record.purposes):
                
                # Check if consent is still fresh
                age = current_time - consent_record.timestamp
                if age < self.consent_refresh_period:
                    return True
                else:
                    # Consent is stale, mark as inactive
                    consent_record.is_active = False
                    logger.warning(f"Consent expired for data subject {data_subject_id}")
        
        return False
    
    def withdraw_consent(self, data_subject_id: str, consent_id: Optional[str] = None) -> bool:
        """Process consent withdrawal"""
        
        withdrawals = 0
        
        for record in self.consent_records.values():
            if record.data_subject_id == data_subject_id:
                if consent_id is None or record.consent_id == consent_id:
                    record.is_active = False
                    record.withdrawal_timestamp = time.time()
                    withdrawals += 1
        
        if withdrawals > 0:
            self._log_processing_activity(
                "consent_withdrawn",
                data_subject_id,
                {"withdrawals": withdrawals, "consent_id": consent_id}
            )
            
            logger.info(f"Consent withdrawn for data subject {data_subject_id}: {withdrawals} records")
            return True
        
        return False
    
    def process_data_subject_request(self, data_subject_id: str, 
                                   request_type: DataSubjectRights,
                                   verification_method: str = "email") -> str:
        """Process data subject rights request"""
        
        request_id = str(uuid.uuid4())
        
        request = DataSubjectRequest(
            request_id=request_id,
            data_subject_id=data_subject_id,
            request_type=request_type,
            timestamp=time.time(),
            status="pending",
            verification_method=verification_method
        )
        
        self.data_subject_requests[request_id] = request
        
        # Log the request
        self._log_processing_activity(
            "data_subject_request",
            data_subject_id,
            {"request_id": request_id, "request_type": request_type.value}
        )
        
        logger.info(f"Data subject request received: {request_type.value} for {data_subject_id}")
        return request_id
    
    def fulfill_access_request(self, request_id: str) -> Optional[Dict[str, Any]]:
        """Fulfill data subject access request (Article 15)"""
        
        request = self.data_subject_requests.get(request_id)
        if not request or request.request_type != DataSubjectRights.ACCESS:
            return None
        
        data_subject_id = request.data_subject_id
        
        # Compile all data for the data subject
        personal_data = {
            "data_subject_id": data_subject_id,
            "consent_records": [],
            "processing_activities": [],
            "request_history": []
        }
        
        # Add consent records
        for consent in self.consent_records.values():
            if consent.data_subject_id == data_subject_id:
                personal_data["consent_records"].append(asdict(consent))
        
        # Add processing activities (where this subject's data is involved)
        for activity in self.processing_activities.values():
            personal_data["processing_activities"].append({
                "purpose": activity.purpose.value,
                "legal_basis": activity.legal_basis.value,
                "retention_period": activity.retention_period,
                "recipients": activity.recipients
            })
        
        # Add request history
        for req in self.data_subject_requests.values():
            if req.data_subject_id == data_subject_id:
                personal_data["request_history"].append(asdict(req))
        
        # Update request status
        request.status = "completed"
        request.response_timestamp = time.time()
        request.data_provided = personal_data
        
        self._log_processing_activity(
            "access_request_fulfilled",
            data_subject_id,
            {"request_id": request_id}
        )
        
        return personal_data
    
    def fulfill_erasure_request(self, request_id: str) -> bool:
        """Fulfill right to be forgotten request (Article 17)"""
        
        request = self.data_subject_requests.get(request_id)
        if not request or request.request_type != DataSubjectRights.ERASURE:
            return False
        
        data_subject_id = request.data_subject_id
        
        # Check if erasure is legally possible
        if not self._can_erase_data(data_subject_id):
            request.status = "denied"
            request.response_timestamp = time.time()
            return False
        
        # Perform erasure
        erasure_results = self._erase_personal_data(data_subject_id)
        
        # Update request status
        request.status = "completed"
        request.response_timestamp = time.time()
        request.data_provided = {"erasure_results": erasure_results}
        
        self._log_processing_activity(
            "erasure_request_fulfilled",
            data_subject_id,
            {"request_id": request_id, "erasure_results": erasure_results}
        )
        
        logger.info(f"Erasure completed for data subject {data_subject_id}")
        return True
    
    def _can_erase_data(self, data_subject_id: str) -> bool:
        """Check if personal data can be erased"""
        
        # Check for legal obligations that prevent erasure
        legal_hold_purposes = [
            ProcessingPurpose.LEGAL_COMPLIANCE,
            ProcessingPurpose.SECURITY  # Security logs may need to be retained
        ]
        
        # In a real implementation, you'd check if data is needed for these purposes
        # For now, we'll assume erasure is generally allowed
        return True
    
    def _erase_personal_data(self, data_subject_id: str) -> Dict[str, int]:
        """Erase personal data for data subject"""
        
        results = {
            "consent_records_removed": 0,
            "processing_logs_anonymized": 0,
            "cached_data_cleared": 0
        }
        
        # Remove consent records
        consent_ids_to_remove = [
            consent_id for consent_id, consent in self.consent_records.items()
            if consent.data_subject_id == data_subject_id
        ]
        
        for consent_id in consent_ids_to_remove:
            del self.consent_records[consent_id]
            results["consent_records_removed"] += 1
        
        # Anonymize processing logs
        for log_entry in self.processing_log:
            if log_entry.get("data_subject_id") == data_subject_id:
                log_entry["data_subject_id"] = self._anonymize_id(data_subject_id)
                results["processing_logs_anonymized"] += 1
        
        # In a real system, you'd also:
        # - Clear cached analysis results
        # - Remove from databases
        # - Notify downstream systems
        
        return results
    
    def _anonymize_id(self, data_subject_id: str) -> str:
        """Create anonymized version of data subject ID"""
        return hashlib.sha256(f"anonymized_{data_subject_id}".encode()).hexdigest()[:16]
    
    def data_portability_export(self, data_subject_id: str, format: str = "json") -> str:
        """Export personal data in structured format (Article 20)"""
        
        # Get all personal data
        access_data = self.fulfill_access_request(
            self.process_data_subject_request(data_subject_id, DataSubjectRights.ACCESS)
        )
        
        if not access_data:
            return ""
        
        # Format for portability
        portable_data = {
            "export_timestamp": time.time(),
            "data_subject_id": data_subject_id,
            "controller": self.controller_name,
            "data": access_data,
            "format_version": "1.0"
        }
        
        if format.lower() == "json":
            return json.dumps(portable_data, indent=2, default=str)
        elif format.lower() == "xml":
            # Simple XML export (would use proper XML library in production)
            return f"<data_export>\\n  <data_subject_id>{data_subject_id}</data_subject_id>\\n  <!-- XML content -->\\n</data_export>"
        else:
            return json.dumps(portable_data, indent=2, default=str)
    
    def _log_processing_activity(self, activity: str, data_subject_id: str, details: Dict[str, Any]):
        """Log processing activity for audit trail"""
        
        log_entry = {
            "timestamp": time.time(),
            "activity": activity,
            "data_subject_id": data_subject_id,
            "details": details,
            "controller": self.controller_name
        }
        
        self.processing_log.append(log_entry)
        
        # Keep log size manageable (in production, would use proper log rotation)
        if len(self.processing_log) > 10000:
            self.processing_log = self.processing_log[-5000:]
    
    def generate_compliance_report(self) -> Dict[str, Any]:
        """Generate GDPR compliance report"""
        
        current_time = time.time()
        
        # Analyze consent records
        active_consents = sum(1 for c in self.consent_records.values() if c.is_active)
        expired_consents = sum(
            1 for c in self.consent_records.values() 
            if not c.is_active or (current_time - c.timestamp) > self.consent_refresh_period
        )
        
        # Analyze data subject requests
        request_stats = {}
        for request_type in DataSubjectRights:
            count = sum(
                1 for r in self.data_subject_requests.values()
                if r.request_type == request_type
            )
            request_stats[request_type.value] = count
        
        # Processing activity summary
        activity_summary = {
            activity.activity_id: {
                "purpose": activity.purpose.value,
                "legal_basis": activity.legal_basis.value,
                "retention_period": activity.retention_period
            }
            for activity in self.processing_activities.values()
        }
        
        report = {
            "controller": self.controller_name,
            "dpo_contact": self.dpo_contact,
            "report_timestamp": current_time,
            "consent_statistics": {
                "active_consents": active_consents,
                "expired_consents": expired_consents,
                "total_consents": len(self.consent_records)
            },
            "data_subject_requests": request_stats,
            "processing_activities": activity_summary,
            "recent_activity": self.processing_log[-100:],  # Last 100 activities
            "compliance_status": "compliant"  # Would include actual compliance checks
        }
        
        return report
    
    def cleanup_expired_data(self) -> Dict[str, int]:
        """Clean up expired data based on retention policies"""
        
        current_time = time.time()
        cleanup_results = {
            "expired_consents_removed": 0,
            "old_logs_archived": 0,
            "completed_requests_archived": 0
        }
        
        # Remove very old expired consents (beyond legal requirement to keep records)
        cutoff_time = current_time - (3 * 365 * 24 * 3600)  # 3 years
        
        expired_consent_ids = [
            consent_id for consent_id, consent in self.consent_records.items()
            if not consent.is_active and consent.timestamp < cutoff_time
        ]
        
        for consent_id in expired_consent_ids:
            del self.consent_records[consent_id]
            cleanup_results["expired_consents_removed"] += 1
        
        # Archive old processing logs
        log_cutoff = current_time - (365 * 24 * 3600)  # 1 year
        old_logs = [log for log in self.processing_log if log["timestamp"] < log_cutoff]
        
        # In production, would move to archive storage
        cleanup_results["old_logs_archived"] = len(old_logs)
        self.processing_log = [log for log in self.processing_log if log["timestamp"] >= log_cutoff]
        
        logger.info(f"Data cleanup completed: {cleanup_results}")
        return cleanup_results
    
    def validate_legal_basis(self, purpose: ProcessingPurpose, legal_basis: LegalBasis) -> bool:
        """Validate that legal basis is appropriate for processing purpose"""
        
        # Define valid combinations
        valid_combinations = {
            ProcessingPurpose.SENTIMENT_ANALYSIS: [
                LegalBasis.CONSENT,
                LegalBasis.LEGITIMATE_INTERESTS,
                LegalBasis.CONTRACT
            ],
            ProcessingPurpose.RESEARCH: [
                LegalBasis.CONSENT,
                LegalBasis.PUBLIC_TASK
            ],
            ProcessingPurpose.SECURITY: [
                LegalBasis.LEGITIMATE_INTERESTS,
                LegalBasis.LEGAL_OBLIGATION
            ],
            ProcessingPurpose.QUALITY_IMPROVEMENT: [
                LegalBasis.LEGITIMATE_INTERESTS,
                LegalBasis.CONSENT
            ],
            ProcessingPurpose.LEGAL_COMPLIANCE: [
                LegalBasis.LEGAL_OBLIGATION
            ]
        }
        
        return legal_basis in valid_combinations.get(purpose, [])